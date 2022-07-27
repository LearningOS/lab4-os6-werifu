//! Implementation of [`Processor`] and Intersection of control flow
//!
//! Here, the continuous operation of user apps in CPU is maintained,
//! the current running state of CPU is recorded,
//! and the replacement and transfer of control flow of different applications are executed.

use super::__switch;
use super::{fetch_task, TaskStatus};
use super::{TaskContext, TaskControlBlock};
use crate::config::PAGE_SIZE;
use crate::mm::{MapPermission, VPNRange, VirtAddr, VirtPageNum};
use crate::sync::UPSafeCell;
use crate::syscall::TaskInfo;
use crate::timer::get_time_ms;
use crate::trap::TrapContext;
use alloc::sync::Arc;
use lazy_static::*;
/// Processor management structure
pub struct Processor {
    /// The task currently executing on the current processor
    current: Option<Arc<TaskControlBlock>>,
    /// The basic control flow of each core, helping to select and switch process
    idle_task_cx: TaskContext,
}

impl Processor {
    pub fn new() -> Self {
        Self {
            current: None,
            idle_task_cx: TaskContext::zero_init(),
        }
    }
    fn get_idle_task_cx_ptr(&mut self) -> *mut TaskContext {
        &mut self.idle_task_cx as *mut _
    }
    pub fn take_current(&mut self) -> Option<Arc<TaskControlBlock>> {
        self.current.take()
    }
    pub fn current(&self) -> Option<Arc<TaskControlBlock>> {
        self.current.as_ref().map(|task| Arc::clone(task))
    }
}

/// for map & unmap
impl Processor {
    fn task_map(&self, start: usize, len: usize, port: usize) -> isize {
        if start & (PAGE_SIZE - 1) != 0 {
            println!(
                "expect the start address to be aligned with a page, but get an invalid start: {:#x}",
                start
            );
            return -1;
        }
        // port最低三位[x w r]，其他位必须为0
        if port > 7usize || port == 0 {
            println!("invalid port: {:#b}", port);
            return -1;
        }

        let task = self.current().unwrap();
        let mut inner = task.inner_exclusive_access();
        let memory_set = &mut inner.memory_set;

        // check valid
        let start_vpn = VirtPageNum::from(VirtAddr(start));
        let end_vpn = VirtPageNum::from(VirtAddr(start + len).ceil());
        for vpn in start_vpn.0..end_vpn.0 {
            if let Some(pte) = memory_set.translate(VirtPageNum(vpn)) {
                if pte.is_valid() {
                    println!("vpn {} has been occupied!", vpn);
                    return -1;
                }
            }
        }

        let permission = MapPermission::from_bits((port as u8) << 1).unwrap() | MapPermission::U;
        memory_set.insert_framed_area(VirtAddr(start), VirtAddr(start + len), permission);
        0
    }

    fn task_munmap(&self, start: usize, len: usize) -> isize {
        if start & (PAGE_SIZE - 1) != 0 {
            println!(
                "expect the start address to be aligned with a page, but get an invalid start: {:#x}",
                start
            );
            return -1;
        }

        let task = self.current().unwrap();
        let mut inner = task.inner_exclusive_access();
        let memory_set = &mut inner.memory_set;

        // check valid
        let start_vpn = VirtPageNum::from(VirtAddr(start));
        let end_vpn = VirtPageNum::from(VirtAddr(start + len).ceil());
        for vpn in start_vpn.0..end_vpn.0 {
            if let Some(pte) = memory_set.translate(VirtPageNum(vpn)) {
                if !pte.is_valid() {
                    println!("vpn {} is not valid before unmap", vpn);
                    return -1;
                }
            }
        }

        let vpn_range = VPNRange::new(start_vpn, end_vpn);
        memory_set.munmap(vpn_range);
        0
    }
}

lazy_static! {
    /// PROCESSOR instance through lazy_static!
    pub static ref PROCESSOR: UPSafeCell<Processor> = unsafe { UPSafeCell::new(Processor::new()) };
}

/// The main part of process execution and scheduling
///
/// Loop fetch_task to get the process that needs to run,
/// and switch the process through __switch
pub fn run_tasks() {
    loop {
        let mut processor = PROCESSOR.exclusive_access();
        if let Some(task) = fetch_task() {
            let idle_task_cx_ptr = processor.get_idle_task_cx_ptr();
            // access coming task TCB exclusively
            let mut task_inner = task.inner_exclusive_access();
            let next_task_cx_ptr = &task_inner.task_cx as *const TaskContext;
            task_inner.task_status = TaskStatus::Running;
            drop(task_inner);
            // release coming task TCB manually
            processor.current = Some(task);
            // release processor manually
            drop(processor);
            unsafe {
                __switch(idle_task_cx_ptr, next_task_cx_ptr);
            }
        }
    }
}

/// Get current task through take, leaving a None in its place
pub fn take_current_task() -> Option<Arc<TaskControlBlock>> {
    PROCESSOR.exclusive_access().take_current()
}

/// Get a copy of the current task
pub fn current_task() -> Option<Arc<TaskControlBlock>> {
    PROCESSOR.exclusive_access().current()
}

/// Get token of the address space of current task
pub fn current_user_token() -> usize {
    let task = current_task().unwrap();
    let token = task.inner_exclusive_access().get_user_token();
    token
}

/// Get the mutable reference to trap context of current task
pub fn current_trap_cx() -> &'static mut TrapContext {
    current_task()
        .unwrap()
        .inner_exclusive_access()
        .get_trap_cx()
}

/// Return to idle control flow for new scheduling
pub fn schedule(switched_task_cx_ptr: *mut TaskContext) {
    let mut processor = PROCESSOR.exclusive_access();
    let idle_task_cx_ptr = processor.get_idle_task_cx_ptr();
    drop(processor);
    unsafe {
        __switch(switched_task_cx_ptr, idle_task_cx_ptr);
    }
}

impl Processor {
    /// syscall_times[id] += 1
    pub fn add_syscall_times(&self, syscall_id: usize) {
        self.current()
            .unwrap()
            .inner_exclusive_access()
            .syscall_times[syscall_id] += 1;
    }
}

/// Record a syscall
pub fn record_syscall(syscall_id: usize) {
    PROCESSOR.exclusive_access().add_syscall_times(syscall_id);
}

/// Write the info of the task into *ti
pub fn get_current_task_info(ti: *mut TaskInfo) {
    let task = current_task().unwrap();
    let inner = task.inner_exclusive_access();
    unsafe {
        *ti = TaskInfo {
            status: TaskStatus::Running,
            syscall_times: inner.syscall_times,
            time: get_time_ms() - inner.start_time,
        };
    }
}

/// for lab. map
pub fn task_mmap(start: usize, len: usize, port: usize) -> isize {
    PROCESSOR.exclusive_access().task_map(start, len, port)
}

/// for lab. unmap
/// for lab. unmap
pub fn task_munmap(start: usize, len: usize) -> isize {
    PROCESSOR.exclusive_access().task_munmap(start, len)
}
