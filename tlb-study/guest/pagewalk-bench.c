/* Deterministic guest-side translation-pressure benchmark. */
#define _GNU_SOURCE
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

static uint64_t nsec(void)
{
    struct timespec ts;

    if (clock_gettime(CLOCK_MONOTONIC, &ts)) {
        perror("clock_gettime");
        exit(EXIT_FAILURE);
    }
    return (uint64_t)ts.tv_sec * 1000000000 + ts.tv_nsec;
}

static size_t parse_size(const char *text, const char *what)
{
    char *end;
    unsigned long long value = strtoull(text, &end, 0);

    if (!text[0] || *end || !value || value > SIZE_MAX) {
        fprintf(stderr, "invalid %s: %s\n", what, text);
        exit(EXIT_FAILURE);
    }
    return value;
}

static size_t anon_huge_kib(uintptr_t start, uintptr_t end)
{
    char line[256];
    bool in_mapping = false;
    size_t huge_kib = 0;
    FILE *fp = fopen("/proc/self/smaps", "r");

    if (!fp) {
        return 0;
    }
    while (fgets(line, sizeof(line), fp)) {
        uintptr_t lo, hi;

        if (sscanf(line, "%" SCNxPTR "-%" SCNxPTR, &lo, &hi) == 2) {
            in_mapping = lo <= start && hi >= end;
        } else if (in_mapping &&
                   sscanf(line, "AnonHugePages: %zu kB", &huge_kib) == 1) {
            break;
        }
    }
    fclose(fp);
    return huge_kib;
}

static void host_barrier(const char *marker)
{
    char line[8];
    FILE *control;

    puts(marker);
    fflush(stdout);
    control = fopen("/dev/ttyS0", "r");
    if (!control || !fgets(line, sizeof(line), control)) {
        perror("host barrier");
        exit(EXIT_FAILURE);
    }
    fclose(control);
}

int main(int argc, char **argv)
{
    const size_t page_size = 4096;
    const char *mode, *page_mode;
    size_t mib, passes, length, pages, pass, i, accesses;
    volatile uint64_t checksum = 0;
    uint8_t *mem;
    uint64_t begin, end;
    int advice;

    if (argc != 5 && argc != 6) {
        fprintf(stderr,
                "usage: %s MODE MIB PASSES huge|nohuge [handshake]\n"
                "MODE is dense, seq, random, conflict, or mprotect\n",
                argv[0]);
        return EXIT_FAILURE;
    }
    mode = argv[1];
    mib = parse_size(argv[2], "MiB");
    passes = parse_size(argv[3], "pass count");
    page_mode = argv[4];
    length = mib * 1024 * 1024;
    pages = length / page_size;
    if (!pages || (pages & (pages - 1))) {
        fprintf(stderr, "MIB must produce a power-of-two page count\n");
        return EXIT_FAILURE;
    }

    mem = mmap(NULL, length, PROT_READ | PROT_WRITE,
               MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (mem == MAP_FAILED) {
        perror("mmap");
        return EXIT_FAILURE;
    }
    if (!strcmp(page_mode, "huge")) {
        advice = MADV_HUGEPAGE;
    } else if (!strcmp(page_mode, "nohuge")) {
        advice = MADV_NOHUGEPAGE;
    } else {
        fprintf(stderr, "page mode must be huge or nohuge\n");
        return EXIT_FAILURE;
    }
    if (madvise(mem, length, advice)) {
        perror("madvise");
        return EXIT_FAILURE;
    }

    for (i = 0; i < pages; i++) {
        mem[i * page_size] = (uint8_t)i;
    }
    printf("mapping=%p-%p mib=%zu pages=%zu anon_huge_kib=%zu mode=%s\n",
           mem, mem + length, mib, pages,
           anon_huge_kib((uintptr_t)mem, (uintptr_t)mem + length), page_mode);
    fflush(stdout);

    if (argc == 6 && !strcmp(argv[5], "handshake")) {
        host_barrier("TLB-BENCH-READY");
    }

    begin = nsec();
    for (pass = 0; pass < passes; pass++) {
        if (!strcmp(mode, "dense")) {
            for (i = 0; i < length; i += 64) {
                checksum += mem[i];
            }
        } else if (!strcmp(mode, "seq")) {
            for (i = 0; i < pages; i++) {
                checksum += mem[i * page_size];
            }
        } else if (!strcmp(mode, "random")) {
            size_t index = pass & (pages - 1);

            for (i = 0; i < pages; i++) {
                index = (index + 0x9e3779b9U) & (pages - 1);
                checksum += mem[index * page_size];
            }
        } else if (!strcmp(mode, "conflict")) {
            const size_t stride_pages = 256;

            for (i = 0; i < pages; i++) {
                size_t index = (i * stride_pages + pass) & (pages - 1);
                checksum += mem[index * page_size];
            }
        } else if (!strcmp(mode, "mprotect")) {
            if (mprotect(mem, length, PROT_READ)) {
                perror("mprotect read");
                return EXIT_FAILURE;
            }
            for (i = 0; i < pages; i++) {
                checksum += mem[i * page_size];
            }
            if (mprotect(mem, length, PROT_READ | PROT_WRITE)) {
                perror("mprotect read-write");
                return EXIT_FAILURE;
            }
        } else {
            fprintf(stderr, "unknown mode: %s\n", mode);
            return EXIT_FAILURE;
        }
    }
    end = nsec();
    accesses = !strcmp(mode, "dense") ? passes * length / 64 : passes * pages;
    printf("result mode=%s mib=%zu passes=%zu elapsed_ns=%" PRIu64
           " accesses=%zu ns_per_access=%.3f checksum=%" PRIu64 "\n",
           mode, mib, passes, end - begin, accesses,
           (double)(end - begin) / accesses, checksum);
    if (argc == 6 && !strcmp(argv[5], "handshake")) {
        host_barrier("TLB-BENCH-DONE");
    }
    return 0;
}
