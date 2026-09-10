#include <stdio.h>
#include <string.h>

__attribute__((noinline, noclone, used))
void tlb_window_start(void)
{
    __asm__ volatile("" ::: "memory");
}

__attribute__((noinline, noclone, used))
void tlb_window_stop(void)
{
    __asm__ volatile("" ::: "memory");
}

int main(int argc, char **argv)
{
    void (*volatile fn)(void);

    if (argc != 2) {
        fprintf(stderr, "usage: %s start|stop\n", argv[0]);
        return 2;
    }
    if (strcmp(argv[1], "start") == 0) {
        fn = tlb_window_start;
    } else if (strcmp(argv[1], "stop") == 0) {
        fn = tlb_window_stop;
    } else {
        fprintf(stderr, "unknown marker: %s\n", argv[1]);
        return 2;
    }
    fn();
    return 0;
}
