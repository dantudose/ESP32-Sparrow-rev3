#include "fs_utils.h"

#include <errno.h>
#include <string.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/printk.h>

#include "app_paths.h"

FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(lfs_storage);

static struct fs_mount_t littlefs_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &lfs_storage,
	.storage_dev = (void *)FIXED_PARTITION_ID(storage_partition),
	.mnt_point = LITTLEFS_MOUNT_POINT,
};

int mount_littlefs(void)
{
	int rc = fs_mount(&littlefs_mnt);

	if (rc < 0) {
		printk("LittleFS mount failed (%d)\n", rc);
		return rc;
	}

	printk("LittleFS mounted at %s\n", littlefs_mnt.mnt_point);
	return 0;
}

int ensure_littlefs_ready(void)
{
	struct fs_dirent entry;
	int rc = fs_stat(LITTLEFS_MOUNT_POINT, &entry);

	if (rc < 0) {
		return rc;
	}

	if (entry.type != FS_DIR_ENTRY_DIR) {
		return -ENOTDIR;
	}

	return 0;
}

int ensure_log_dir(void)
{
	struct fs_dirent entry;
	int rc = fs_stat(BME_LOG_DIR, &entry);

	if (rc == 0) {
		if (entry.type != FS_DIR_ENTRY_DIR) {
			return -ENOTDIR;
		}
		return 0;
	}

	if (rc != -ENOENT) {
		return rc;
	}

	rc = fs_mkdir(BME_LOG_DIR);
	if (rc < 0 && rc != -EEXIST) {
		return rc;
	}

	return 0;
}

static int ensure_dir(const char *path)
{
	struct fs_dirent entry;
	int rc = fs_stat(path, &entry);

	if (rc == 0) {
		if (entry.type != FS_DIR_ENTRY_DIR) {
			return -ENOTDIR;
		}
		return 0;
	}

	if (rc != -ENOENT) {
		return rc;
	}

	rc = fs_mkdir(path);
	if (rc < 0 && rc != -EEXIST) {
		return rc;
	}

	return 0;
}

static int ensure_dir_tree(const char *dir)
{
	char tmp[BME_LOG_PATH_MAX];
	size_t len;

	if (!dir || dir[0] != '/') {
		return -EINVAL;
	}

	if (strlen(dir) >= sizeof(tmp)) {
		return -ENAMETOOLONG;
	}

	len = strlen(dir);
	memcpy(tmp, dir, len + 1);
	for (char *cur = tmp + 1; *cur != '\0'; cur++) {
		if (*cur == '/') {
			*cur = '\0';
			int rc = ensure_dir(tmp);
			if (rc < 0) {
				return rc;
			}
			*cur = '/';
		}
	}

	return ensure_dir(tmp);
}

int ensure_dir_tree_for_file(const char *path)
{
	char dir[BME_LOG_PATH_MAX];
	size_t len;
	char *slash;

	if (!path || path[0] != '/') {
		return -EINVAL;
	}

	if (strlen(path) >= sizeof(dir)) {
		return -ENAMETOOLONG;
	}

	len = strlen(path);
	memcpy(dir, path, len + 1);
	slash = strrchr(dir, '/');
	if (!slash || slash == dir) {
		return 0;
	}

	if (slash[1] == '\0') {
		return -EINVAL;
	}

	*slash = '\0';
	return ensure_dir_tree(dir);
}
