#ifndef FLASH_STORAGE_H
#define FLASH_STORAGE_H

typedef void (*read_data_cb_t)(void* data,int len);

int flash_storage_init(read_data_cb_t read_data_cb);
int flash_storage_store(void* data,int len);
int flash_storage_load(void* data,int len);

#endif /* FLASH_STORAGE_H */