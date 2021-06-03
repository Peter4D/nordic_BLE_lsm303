#include "fds.h"
#include "flash_storage.h"
#include "nrf_log.h"

#define FILE_ID 0x4B1D // Forbidden, heh
#define KEY_ID  0xBEEF // Beef is good

static uint8_t storage[sizeof(uint32_t)] __ALIGN(sizeof(uint32_t));

static fds_record_t record = {
  .file_id = FILE_ID,
  .key = KEY_ID,
  .data.p_data = storage,
  .data.length_words = 1
};

static fds_record_desc_t record_desc;

static read_data_cb_t read_data_cb;

static void fds_evt_handler(fds_evt_t const * p_fds_evt)
{
    switch (p_fds_evt->id)
    {
        case FDS_EVT_INIT:
            APP_ERROR_CHECK(p_fds_evt->result);
            break;
        case FDS_EVT_GC:  
            if(p_fds_evt->result == NRF_SUCCESS){
              NRF_LOG_INFO("GC RAN");
            }
            break;
        default:
            break;
    }
}

int flash_storage_init(read_data_cb_t rd_cb)
{
  ASSERT(rd_cb != NULL);
  read_data_cb = rd_cb;

  ret_code_t ret = fds_register(fds_evt_handler);
  if (ret != NRF_SUCCESS)
  {
      return -1;
  }
  ret = fds_init();
  if (ret != NRF_SUCCESS)
  {
      return -1;
  }
  fds_find_token_t token = {};
  ret = fds_record_find(FILE_ID,KEY_ID,&record_desc,&token);
  if(ret != NRF_SUCCESS){
    ret = fds_record_write(&record_desc,&record);
    fds_gc();
  }
}

int flash_storage_store(void* data,int len)
{
  ASSERT(len <= 4);
  memcpy(storage,data,len);
  fds_find_token_t token = {};
  ret_code_t ret = fds_record_find(FILE_ID,KEY_ID,&record_desc,&token);
  APP_ERROR_CHECK(ret);
  ret = fds_record_update(&record_desc,&record);
  APP_ERROR_CHECK(ret);
  fds_gc();
}

int flash_storage_load(void* data,int len)
{
  ASSERT(len <= 4);
  fds_find_token_t token = {};
  ret_code_t ret = fds_record_find(FILE_ID,KEY_ID,&record_desc,&token);
  APP_ERROR_CHECK(ret);
  fds_record_t rec = {};
  APP_ERROR_CHECK(fds_record_open(&record_desc,&rec));
  memcpy(data,rec.data.p_data,len);
  fds_record_close(&record_desc);
}

int flash_storage_clear()
{
  
}