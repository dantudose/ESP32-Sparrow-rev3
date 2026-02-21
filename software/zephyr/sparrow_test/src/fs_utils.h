#pragma once

int mount_littlefs(void);
int ensure_littlefs_ready(void);
int ensure_log_dir(void);
int ensure_dir_tree_for_file(const char *path);
