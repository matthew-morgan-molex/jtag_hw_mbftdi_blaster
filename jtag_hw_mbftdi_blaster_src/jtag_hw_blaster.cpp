//
// jtag_hw_blaster.cpp:  altera/intel jtagsrv blaster helper DLL.
//
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <unistd.h>
#include <ctype.h>
#include <stdarg.h>

#include "debug.h"
#include "jtagsrv.h"
#include "CConfig.h"
#include "jtag_hw_blaster.h"

struct jtagsrv_interface jtagsrvi;
jblaster* g_pblaster[MAX_DEV_NUM];

jblaster::jblaster( int idx ):m_dev_idx(idx)
{
};

jblaster::~jblaster()
{
};

void jblaster::alloc_jtag_task(unsigned int count, unsigned int need_tdo)
{
    int buf_size = max((int)count + MIN_TASK_BUF_SIZE/2, MIN_TASK_BUF_SIZE);
    int jtask_mem_size = sizeof(struct jtag_task) + buf_size;
    jtask_ = (struct jtag_task*)new char[jtask_mem_size];
    jtask_->buf_size = buf_size;
    jtask_->need_tdo = need_tdo;
    jtask_->wr_idx = 0;
}

void jblaster::realloc_jtag_task(unsigned int count)
{
    int new_buf_size = jtask_->buf_size + max((int)count + MIN_TASK_BUF_SIZE / 2, MIN_TASK_BUF_SIZE);
    int jtask_mem_size = sizeof(struct jtag_task) + new_buf_size;
    struct jtag_task* jt = (struct jtag_task*)new char[jtask_mem_size];
    jt->buf_size = new_buf_size;
    jt->need_tdo = jtask_->need_tdo;
    jt->wr_idx = jtask_->wr_idx;
    memcpy(jt->data, jtask_->data, jtask_->wr_idx);
    char* ptr = (char*)jtask_;
    delete[]ptr;
    jtask_ = jt;
}

void jblaster::check_jtag_task( unsigned int count, unsigned int idx )
{
    unsigned int need_tdo = 0;
    if (jtagsrvi.jtagsrv_get_bits_attr) {
        uint32_t r = jtagsrvi.jtagsrv_get_bits_attr((void*)jtagsrv_context_, idx);
        if ((r & 0x80000000) == 0)
            need_tdo = 1;
    }

    if (jtask_) {
        //jtask already started
        //is task of same type?
        if (jtask_->need_tdo == need_tdo) {
            //same type task, but has space enough?
            unsigned int space = jtask_->buf_size - jtask_->wr_idx;
            if (space < count) {
                //need more space
                realloc_jtag_task(count);
            }
            else {
                //continue existing jtask..
            }
        }
        else {
            //another type of task, so put existing task into queue
            jtag_queue_.push_back(jtask_);
            //start new task of new current type
            alloc_jtag_task(count, need_tdo);
        }
    }
    else {
        //no jtask yet, so create it!
        alloc_jtag_task(count, need_tdo);
    }
}

void jblaster::write_pattern(char tms, char tdi, unsigned int count, unsigned int idx)
{
    check_jtag_task(count, idx);
    char c = 0;
    if (tms)
        c |= TMS_BIT;
    if (tdi)
        c |= TDI_BIT;
    for (unsigned int i = 0; i < count; i++)
    {
        jtask_->data[jtask_->wr_idx++] = c;
    }
    num_bits_in_queue_ += count;
}

unsigned int jblaster::write_masked(char tms, uint32_t* ptdibitarray, unsigned int count, unsigned int idx)
{
    printd("\nwrite_masked: %d %d %d\n", tms, count, idx);
    check_jtag_task(count, idx);
    for (unsigned int i = 0; i < count; i++)
    {
        uint32_t dw, mask;
        char tdi;
        dw = ptdibitarray[i / 32];
        mask = 1 << (i & 0x1f);
        if (dw & mask)
            tdi = 1;
        else
            tdi = 0;
        char c = 0;
        if (tms)
            c |= TMS_BIT;
        if (tdi)
            c |= TDI_BIT;
        jtask_->data[jtask_->wr_idx++] = c;
    }
    num_bits_in_queue_ += count;
    return 0;
}

unsigned int jblaster::send_recv(unsigned int need_rdata)
{
    int r = 1;
    if (jtask_)
    {
        jtag_queue_.push_back(jtask_);
        jtask_ = { nullptr };
    }

    if (last_bits_flags_org_ == 0x13)
    {
        while (!jtag_queue_.empty())
        {
            struct jtag_task* jt = jtag_queue_.front();
            //write_jtag_stream(jt); //jtag mode
            jt->need_tdo = 1;
            write_jtag_stream_as(jt); //active serial
            jtag_queue_.pop_front();
            num_bits_in_queue_ -= jt->wr_idx;
            delete[]jt;
        }
    }
    else
    {
        while (!jtag_queue_.empty())
        {
            struct jtag_task* jt = jtag_queue_.front();
            //jt->need_tdo = 1;
            write_jtag_stream( jt ); //jtag mode
            jtag_queue_.pop_front();
            num_bits_in_queue_ -= jt->wr_idx;
            delete[]jt;
        }
    }

    m_curr_idx = 0;
    num_bits_in_queue_ = 0;
    return r;
}

unsigned int jblaster::print_tdi_tms(const unsigned char* buf, int num_bits)
{
    unsigned char dbg[16];
    unsigned char c = 0;
    memset(dbg, 0, sizeof(dbg));
    int num = min(num_bits,64);
    for (int i = 0; i < num; i++)
    {
        if ((i & 3) == 0)
            c = buf[i];
        else
            if ((i & 3) == 1)
                c = c | (buf[i]<<2);
            else
                if ((i & 3) == 2)
                    c = c | (buf[i] << 4);
                else
                    if ((i & 3) == 3)
                        c = c | (buf[i] << 6);
        dbg[i / 4] = c;
    }
    printd("tms-tdi (%d): %02X %02X %02X %02X %02X %02X\n", num_bits, dbg[0], dbg[1], dbg[2], dbg[3], dbg[4], dbg[5] );
    return 0;
}

unsigned int jblaster::checksum(const unsigned char* buf, int num_bits )
{
    int k;
    int len = (num_bits + 7) / 8;
    unsigned int POLY = 0x82f63b78;
    unsigned int crc = 0;
    while (len--)
    {
        crc ^= *buf++;
        for (k = 0; k < 8; k++)
            crc = crc & 1 ? (crc >> 1) ^ POLY : crc >> 1;
    }
    return ~crc;
}

//------- HWPROC -------

struct hw_descriptor my_hw_descriptor;

//jtag server enumerates attached blasters via this function
//port_num -> varies from 0 and up until function returns zero
//pblaster_name -> dest name ptr like "USB-0", should be filled by function with existed blaster name
//blaster_name_sz -> dest buffer size
unsigned int hwproc_listdev(int port_num, char* pblaster_name, int blaster_name_sz)
{
    //do not support more then 4 blasters
    if (port_num>MAX_DEV_NUM - 1)
        return 0;

    //do we have ftdi attached?
    int numchips = search_blasters(port_num, pblaster_name, blaster_name_sz);
    if (numchips <= 0)
        return 0;       //no FTDI found or error

    if (port_num >= numchips)
        return 0;       //no more FTDI chips

    return 1;
}

int g_num_open = 0;

//called by jtagsrv opening desired blaster
int hwproc_open_init(void**                    pmycontext,       // put my context pointer here
                     char*                     pdevname,         // string like "USB-0"
                     struct jtagsrv_interface* pjtagsrv_struct,  // jtagsrv struct has pointers to some usefull functions
                     void*                     jtagsrv_context)  // jtagsrv functions context
{
    //result OK
    int r = 0x56;
    int status;
    int dev_idx = port_name_2_idx(pdevname);

    if (dev_idx < 0)
    {
        return 0;
    }

    printd("------------------ (%d) open jtag blaster %d %s --------------------\n", g_num_open, dev_idx, pdevname);

    jblaster* pblaster = g_pblaster[dev_idx] = create_blaster(dev_idx);
    if (pblaster == nullptr)
        return 0;
    printd("pblaster context = %p\n", pblaster);
    
    status = pblaster->configure();
    if (status == 0)
    {
        printd("cannot configure MPSSE %d %s\n", dev_idx, pdevname);
        return 0;
    }

    //give them our context
    *pmycontext = (void*)pblaster;

    //save jtagsrv interface struct
    unsigned int jtagsrvi_sz = sizeof(jtagsrvi);
    if (jtagsrvi_sz > pjtagsrv_struct->size)
        jtagsrvi_sz = pjtagsrv_struct->size;
    memcpy(&jtagsrvi, pjtagsrv_struct, jtagsrvi_sz);

    //save jtagsrv context
    pblaster->jtagsrv_context_ = jtagsrv_context;
    return r;
}

unsigned int hwproc_close(unsigned int* pmycontext)
{
    jblaster* pblaster = (jblaster*)pmycontext;
    printd("hwproc_close %p\n", pmycontext);
    delete_blaster(pblaster);
    return 0;
}

unsigned int hwproc_set_param(void* context, char* key, unsigned int value)
{
    jblaster* pblaster = (jblaster*)context;
    unsigned int r = 0;
    printd("hwproc_set_param (%p) %s=%d\n", context, key, value);
    r = pblaster->set_config_value(key, value);
    return r;
}

unsigned int hwproc_get_param(void* context, char* key, unsigned int* value)
{
    jblaster* pblaster = (jblaster*)context;
    unsigned int r = 0;
    printd("hwproc_get_param (%p) %s\n", context, key);
    r = pblaster->get_config_value(key, value);
    printd("     hwproc_get_param ret = %d, value = %d\n", r, *value);
    return r;
}

unsigned int hwproc_unkn00(void* a, unsigned int b, unsigned int c, unsigned int d, unsigned int e)
{
    int r = 0;
    printd("hwproc_unkn00 %p %08X %08X %08X %08X\n", a, b, c, d, e);
    return r;
}
unsigned int hwproc_unkn01(void* a, unsigned int b, unsigned int c, unsigned int d, unsigned int e)
{
    int r = 0;
    printd("hwproc_unkn01 %p %08X %08X %08X %08X\n", a, b, c, d, e);
    return r;
}
unsigned int hwproc_unkn03(void* a, unsigned int b, unsigned int c, unsigned int d, unsigned int e)
{
    int r = 0;
    printd("hwproc_unkn03 %p %08X %08X %08X %08X\n", a, b, c, d, e);
    return r;
}

unsigned int hwproc_unkn11(void* a, unsigned int b, unsigned int c, unsigned int d, unsigned int e)
{
    int r=0;
    printd("hwproc_unkn11 %p %08X %08X %08X %08X\n",a,b,c,d,e);
    return r;
}

unsigned int hwproc_send_recv(void* context, unsigned int need_rdata)
{
    jblaster* pblaster = (jblaster*)context; 
    unsigned int r = pblaster->send_recv(need_rdata);
    return r;
}

//with UsbBlaster this fnction writes into programmer single byte command and reads one byte status
unsigned int hwproc_write_flags_read_status(void* context, unsigned int flags, unsigned int* pstatus)
{
    jblaster* pblaster = (jblaster*)context; 
    printd("hwproc_write_flags_read_status %p %08X %p\n",context,flags,pstatus);
    pblaster->write_flags_read_status(flags, pstatus);
    printd("status: %08X\n", *pstatus);
    return 0;
}


unsigned int hwproc_write_pattern(void* context, unsigned int tms, unsigned int tdi, unsigned int count, unsigned int idx)
{
    jblaster* pblaster = (jblaster*)context; 
    pblaster->write_pattern(tms,tdi,count,idx);
    return 0;
}

unsigned int hwproc_write_masked(void* context, unsigned int tms, unsigned int* ptdibitarray, unsigned int count, unsigned int idx)
{
    jblaster* pblaster = (jblaster*)context; 
    pblaster->write_masked(tms, ptdibitarray, count, idx);
    return 0;
}

void init_my_hw_descr()
{
    printd("init_my_hw_descr: %p\n", &my_hw_descriptor);

    memset((void*)&my_hw_descriptor, 0, sizeof(struct hw_descriptor));
    my_hw_descriptor.size = sizeof(my_hw_descriptor);
    strncpy  ((char*)&my_hw_descriptor.hw_name[0], get_blaster_name(), sizeof(my_hw_descriptor.hw_name) - 1);
    my_hw_descriptor.unknown = 0x3802; //0x800
    my_hw_descriptor._hwproc_listdev  = (pfunc_listdev)hwproc_listdev;
    my_hw_descriptor._hwproc_open_init = (pfunc_open_init)hwproc_open_init;
    my_hw_descriptor._hwproc_close = (pfunc_close)hwproc_close;
    my_hw_descriptor._hwproc_set_param = (pfunc_set_param)hwproc_set_param;
    my_hw_descriptor._hwproc_get_param = (pfunc_get_param)hwproc_get_param;
    my_hw_descriptor._hwproc_write_flags_read_status = (pfunc_write_flags_read_status)hwproc_write_flags_read_status;
    my_hw_descriptor._hwproc_write_pattern = (pfunc_write_pattern)hwproc_write_pattern;
    my_hw_descriptor._hwproc_write_masked = (pfunc_write_masked)hwproc_write_masked;
    my_hw_descriptor._hwproc_send_recv = (pfunc_send_recv)hwproc_send_recv;

    //my_hw_descriptor.func00 = (FUNCx)hwproc_unkn00;
    //my_hw_descriptor.func01 = (FUNCx)hwproc_unkn01;
    //my_hw_descriptor.func03 = (FUNCx)hwproc_unkn03;
    //my_hw_descriptor.func11 = (FUNCx)hwproc_unkn11;
}

extern "C" struct hw_descriptor* get_supported_hardware(int hw_id)
{
    printd("Hello MBFTDI JTAG!\n");
#if _DEBUG
    printd("get_supported_hardware was called!!! %08X\n", hw_id);
#endif
    if(hw_id > 0)
        return nullptr;

    init_my_hw_descr();
    return &my_hw_descriptor;
}
