/* Count guest memory operations between two marker instruction addresses. */
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <glib.h>
#include <qemu-plugin.h>

QEMU_PLUGIN_EXPORT int qemu_plugin_version = QEMU_PLUGIN_VERSION;

static uint64_t total_mem;
static uint64_t window_start;
static uint64_t window_mem;
static uint64_t start_pc;
static uint64_t stop_pc;
static unsigned int windows;
static bool active;

static void report(void)
{
    g_autoptr(GString) out = g_string_new("");

    g_string_printf(out,
                    "window mem accesses: %" PRIu64 " windows: %u active: %u\n",
                    window_mem, windows, active);
    qemu_plugin_outs(out->str);
}

static void marker(unsigned int cpu_index, void *userdata)
{
    bool is_start = (uintptr_t)userdata;

    (void)cpu_index;
    if (is_start) {
        window_start = total_mem;
        active = true;
    } else if (active) {
        window_mem += total_mem - window_start;
        windows++;
        active = false;
        /* System emulation may terminate without reaching the exit callback. */
        report();
    }
}

static void translate(qemu_plugin_id_t id, struct qemu_plugin_tb *tb)
{
    size_t n = qemu_plugin_tb_n_insns(tb);

    (void)id;
    for (size_t i = 0; i < n; i++) {
        struct qemu_plugin_insn *insn = qemu_plugin_tb_get_insn(tb, i);
        uint64_t pc = qemu_plugin_insn_vaddr(insn);

        qemu_plugin_register_vcpu_mem_inline(insn, QEMU_PLUGIN_MEM_RW,
                                             QEMU_PLUGIN_INLINE_ADD_U64,
                                             &total_mem, 1);
        if (pc == start_pc || pc == stop_pc) {
            qemu_plugin_register_vcpu_insn_exec_cb(
                insn, marker, QEMU_PLUGIN_CB_NO_REGS,
                (void *)(uintptr_t)(pc == start_pc));
        }
    }
}

static void plugin_exit(qemu_plugin_id_t id, void *userdata)
{
    (void)id;
    (void)userdata;
    report();
}

static bool parse_pc(const char *arg, const char *name, uint64_t *value)
{
    size_t len = strlen(name);
    char *end;

    if (strncmp(arg, name, len) != 0 || arg[len] != '=') {
        return false;
    }
    *value = strtoull(arg + len + 1, &end, 0);
    return end != arg + len + 1 && *end == '\0';
}

QEMU_PLUGIN_EXPORT int qemu_plugin_install(qemu_plugin_id_t id,
                                           const qemu_info_t *info,
                                           int argc, char **argv)
{
    bool have_start = false;
    bool have_stop = false;

    (void)info;
    for (int i = 0; i < argc; i++) {
        if (parse_pc(argv[i], "start", &start_pc)) {
            have_start = true;
        } else if (parse_pc(argv[i], "stop", &stop_pc)) {
            have_stop = true;
        } else {
            fprintf(stderr, "mem-window: invalid option: %s\n", argv[i]);
            return -1;
        }
    }
    if (!have_start || !have_stop || start_pc == stop_pc) {
        fprintf(stderr, "mem-window: require distinct start=<pc>,stop=<pc>\n");
        return -1;
    }
    qemu_plugin_register_vcpu_tb_trans_cb(id, translate);
    qemu_plugin_register_atexit_cb(id, plugin_exit, NULL);
    return 0;
}
