
    .align 3
    .section .data
    .global _num_app
_num_app:
    .quad 23
    .quad app_0_start
    .quad app_1_start
    .quad app_2_start
    .quad app_3_start
    .quad app_4_start
    .quad app_5_start
    .quad app_6_start
    .quad app_7_start
    .quad app_8_start
    .quad app_9_start
    .quad app_10_start
    .quad app_11_start
    .quad app_12_start
    .quad app_13_start
    .quad app_14_start
    .quad app_15_start
    .quad app_16_start
    .quad app_17_start
    .quad app_18_start
    .quad app_19_start
    .quad app_20_start
    .quad app_21_start
    .quad app_22_start
    .quad app_22_end

    .section .data
    .global app_0_start
    .global app_0_end
app_0_start:
    .incbin "../user/build/bin/ch2b_bad_address.bin"
app_0_end:

    .section .data
    .global app_1_start
    .global app_1_end
app_1_start:
    .incbin "../user/build/bin/ch2b_bad_instructions.bin"
app_1_end:

    .section .data
    .global app_2_start
    .global app_2_end
app_2_start:
    .incbin "../user/build/bin/ch2b_bad_register.bin"
app_2_end:

    .section .data
    .global app_3_start
    .global app_3_end
app_3_start:
    .incbin "../user/build/bin/ch2b_hello_world.bin"
app_3_end:

    .section .data
    .global app_4_start
    .global app_4_end
app_4_start:
    .incbin "../user/build/bin/ch2b_power_3.bin"
app_4_end:

    .section .data
    .global app_5_start
    .global app_5_end
app_5_start:
    .incbin "../user/build/bin/ch2b_power_5.bin"
app_5_end:

    .section .data
    .global app_6_start
    .global app_6_end
app_6_start:
    .incbin "../user/build/bin/ch2b_power_7.bin"
app_6_end:

    .section .data
    .global app_7_start
    .global app_7_end
app_7_start:
    .incbin "../user/build/bin/ch3b_sleep.bin"
app_7_end:

    .section .data
    .global app_8_start
    .global app_8_end
app_8_start:
    .incbin "../user/build/bin/ch3b_sleep1.bin"
app_8_end:

    .section .data
    .global app_9_start
    .global app_9_end
app_9_start:
    .incbin "../user/build/bin/ch3b_yield0.bin"
app_9_end:

    .section .data
    .global app_10_start
    .global app_10_end
app_10_start:
    .incbin "../user/build/bin/ch3b_yield1.bin"
app_10_end:

    .section .data
    .global app_11_start
    .global app_11_end
app_11_start:
    .incbin "../user/build/bin/ch3b_yield2.bin"
app_11_end:

    .section .data
    .global app_12_start
    .global app_12_end
app_12_start:
    .incbin "../user/build/bin/ch5b_exit.bin"
app_12_end:

    .section .data
    .global app_13_start
    .global app_13_end
app_13_start:
    .incbin "../user/build/bin/ch5b_forktest.bin"
app_13_end:

    .section .data
    .global app_14_start
    .global app_14_end
app_14_start:
    .incbin "../user/build/bin/ch5b_forktest2.bin"
app_14_end:

    .section .data
    .global app_15_start
    .global app_15_end
app_15_start:
    .incbin "../user/build/bin/ch5b_forktest_simple.bin"
app_15_end:

    .section .data
    .global app_16_start
    .global app_16_end
app_16_start:
    .incbin "../user/build/bin/ch5b_forktree.bin"
app_16_end:

    .section .data
    .global app_17_start
    .global app_17_end
app_17_start:
    .incbin "../user/build/bin/ch5b_initproc.bin"
app_17_end:

    .section .data
    .global app_18_start
    .global app_18_end
app_18_start:
    .incbin "../user/build/bin/ch5b_user_shell.bin"
app_18_end:

    .section .data
    .global app_19_start
    .global app_19_end
app_19_start:
    .incbin "../user/build/bin/ch6b_cat.bin"
app_19_end:

    .section .data
    .global app_20_start
    .global app_20_end
app_20_start:
    .incbin "../user/build/bin/ch6b_filetest_simple.bin"
app_20_end:

    .section .data
    .global app_21_start
    .global app_21_end
app_21_start:
    .incbin "../user/build/bin/ch6b_initproc.bin"
app_21_end:

    .section .data
    .global app_22_start
    .global app_22_end
app_22_start:
    .incbin "../user/build/bin/ch6b_user_shell.bin"
app_22_end:
