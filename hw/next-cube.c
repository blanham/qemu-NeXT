/*
 *  Next Cube System Driver
 *  Copyright (c) 2011 Bryce Lanham
 *
 *  Based on dummy_m68k.c Copyright (c) 2007 CodeSourcery.
 *  TODO:
 *  FIX CODING STYLE!
 *  Change opaques to pass a dynamically allocated machine state
 *  which include the cpu env
 *  Add more comments!
 *
 *  This code is licensed under the GPL
 */


#include "hw.h"
#include "next-cube.h"
#include "exec-all.h"
#include "monitor.h"
#include "sysemu.h"
#include "boards.h"
#include "console.h"
#include "loader.h"
#include "elf.h"
#include "esp.h" //SCSI ESP should work out of the box
#include "esp-new.h"//about that....
#include "sysbus.h"
#include "escc.h" //ZILOG 8530 Serial Emulation
#include "fdc.h"

#include "next-proto.h"

#define ENTRY       0x0100001e
#define RAM_SIZE    0x4000000
#define ROM_FILE    "./rom66.bin"

/* would like to dynamically allocate these later */
next_state_t next_state;
nextfb_state_t nextfb_state;

/* debug NeXT */
#define DEBUG_NEXT

#ifdef DEBUG_NEXT
#define DPRINTF(fmt, ...)                                       \
    do { printf("NeXT: " fmt , ## __VA_ARGS__); } while (0);
#else
#define DPRINTF(fmt, ...) ;
#endif


/* these need to be in machine state */
uint32_t scr1 = 0;
uint32_t scr2 = 0;
uint32_t int_status = 4;//from previous
uint32_t int_mask = 0;
uint32_t event_test = 0;

/* Thanks to NeXT forums for this */
uint8_t rtc_ram3[32]={
0x94,0x0f,0x40,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0xfb,0x6d,0x00,0x00,0x7B,0x00,
0x00,0x00,0x65,0x6e,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x50,0x13
};
uint8_t *rtc_ram;
uint8_t rtc_ram2[32]={
0x94,0x0f,0x40,0x03,0x00,0x00,0x00,0x00,
0x00,0x00,0xfb,0x6d,0x00,0x00,0x4b,0x00,
0x41,0x00,0x20,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x84,0x7e,
};



static uint32_t mmio_readb(void*opaque, target_phys_addr_t addr)
{  
    switch(addr)
    {
        case 0xc000:
            return (scr1 >> 24) & 0xFF;
        case 0xc001:
            return (scr1 >> 16) & 0xFF;
        case 0xc002:
            return (scr1 >> 8)  & 0xFF;
        case 0xc003:
            return (scr1 >> 0)  & 0xFF;
        
        case 0xd000:
            return (scr2 >> 24) & 0xFF;
        case 0xd001:
            return (scr2 >> 16) & 0xFF;
        case 0xd002:
            return (scr2 >> 8)  & 0xFF;
        case 0xd003:
            return (scr2 >> 0)  & 0xFF;
         case 0x14020:
            fprintf(stderr,"Read 0x4020\n");
            return 0x7f;
        
        default:
            fprintf(stderr,"MMIO Read B @ %x\n",addr);
            return 0x0;
    }
}
static uint32_t mmio_readw(void*opaque, target_phys_addr_t addr)
{
    switch(addr)
    {
        default:
            DPRINTF("MMIO Read W @ %x\n",addr);
            return 0x0;
    }
}

static uint32_t mmio_readl(void*opaque, target_phys_addr_t addr)
{
    #ifdef DEBUG_NEXT
    CPUState *s = (CPUState *)opaque;
    #endif
    switch(addr)
    {
        case 0x7000:
            DPRINTF("Read INT status: %x @ %x\n",int_status,s->pc);
            return int_status;
    
        case 0x7800:
            DPRINTF("MMIO Read INT mask: %x @ %x\n",int_mask,s->pc);
            return int_mask;
    
        case 0xc000:
            return scr1;
   
        //case 0xc800:
            // return 0x01000000;
         
        case 0xd000:
            return scr2;
             
        default:
            DPRINTF("MMIO Read L @ %x\n",addr);
            return 0x0;
    }
}
/* this could use a lot of refactoring */
static void nextscr2_write(void *opaque, uint32_t val, int size)
{
   static int led = 0;
    static int phase = 0;
    static uint8_t old_scr2;
    static uint8_t rtc_command = 0;
    static uint8_t rtc_value = 0;
    static uint8_t rtc_status = 0x90;
    static uint8_t rtc_return = 0;
    uint8_t scr2_2; 
 


    if(size == 4)
        scr2_2 = (val >> 8) & 0xFF;
    else
        scr2_2 = val&0xFF;


            if(val &0x1)
            {          
                printf("fault!\n");
                led++;
                if(led == 10)
                {
                    fprintf(stderr,"LED flashing, possible fault, pausing emulation\n");
                    led = 0;
                    //vm_stop(VMSTOP_DEBUG);
                }

            }
            
        if (scr2_2& 0x1) {
        //  fprintf(stderr,"RTC %x phase %i\n",scr2_2,phase);
            if (phase==-1) phase=0;
                // if we are in going down clock... do something
                #define SCR2_RTCLK 0x2
                #define SCR2_RTDATA 0x4
                if (((old_scr2&SCR2_RTCLK)!=(scr2_2&SCR2_RTCLK)) && ((scr2_2&SCR2_RTCLK)==0) ) {
                    if (phase<8)
                        rtc_command=(rtc_command<<1)|((scr2_2&SCR2_RTDATA)?1:0);
                    if ((phase>=8) && (phase<16)) {
                        rtc_value=(rtc_value<<1)|((scr2_2&SCR2_RTDATA)?1:0);
            
                    // if we read RAM register, output RT_DATA bit
                    if (rtc_command<=0x1F) {
                        scr2_2=scr2_2&(~SCR2_RTDATA);
                    if (rtc_ram[rtc_command]&(0x80>>(phase-8)))
                        scr2_2 |=SCR2_RTDATA;
                
                    rtc_return=(rtc_return<<1)|((scr2_2&SCR2_RTDATA)?1:0);
                }
                    // read the status 0x30
                    if (rtc_command==0x30) {
                        scr2_2=scr2_2&(~SCR2_RTDATA);
                        // for now status = 0x98 (new rtc + FTU)
                        if (rtc_status&(0x80>>(phase-8)))
                            scr2_2|=SCR2_RTDATA;
                    
                        rtc_return=(rtc_return<<1)|((scr2_2&SCR2_RTDATA)?1:0);
                    }
                    // read the status 0x31
                    if (rtc_command==0x31) {
                        scr2_2=scr2_2&(~SCR2_RTDATA);
                    // for now 0x00
                    if (0x00&(0x80>>(phase-8)))
                        scr2_2|=SCR2_RTDATA;
                    rtc_return=(rtc_return<<1)|((scr2_2&SCR2_RTDATA)?1:0);
                }
            
                if ((rtc_command>=0x20) && (rtc_command<=0x2F)) {
                    scr2_2=scr2_2&(~SCR2_RTDATA);
                    // for now 0x00
                     time_t time_h = time(NULL);
                    struct tm *info = localtime(&time_h);
  
                    int ret = 0;
                    #define SCR2_TOBCD(x) (((x / 10) << 4) + ( x % 10)) 

                switch(rtc_command)
                {
                    case 0x20:
                        ret = SCR2_TOBCD(info->tm_sec);
                        break;
                    case 0x21:
                        ret = SCR2_TOBCD(info->tm_min);
                        break;
                    case 0x22:
                        ret = SCR2_TOBCD(info->tm_hour);
                        break;
                    case 0x24:
                        ret = SCR2_TOBCD(info->tm_mday);
                        break;
                    case 0x25:
                        ret = SCR2_TOBCD((info->tm_mon+1));
                        break;
                    case 0x26:
                        ret = SCR2_TOBCD((info->tm_year - 100));
                        break;
                        
                }

                    if (ret&(0x80>>(phase-8)))
                        scr2_2|=SCR2_RTDATA;
                    rtc_return=(rtc_return<<1)|((scr2_2&SCR2_RTDATA)?1:0);
                }
            
            }
        
            phase++;
                if (phase==16) {
        //  fprintf(stderr,"SCR2 RTC command complete %x %x %x at PC=$%08x\n",
        //  rtc_command,rtc_value,rtc_return,0);
                        if ((rtc_command>=0x80) && (rtc_command<=0x9F))
                        {
                            rtc_ram[rtc_command-0x80]=rtc_value;
                            //#ifdef READ_RTC
                            FILE *fp = fopen("rtc.ram","wb+");
                            int ret = fwrite(rtc_ram,1,32,fp);
                            if(ret != 32)
                                    abort();
                            fclose(fp);
                            //#endif
                        }
                        // write to x30 register
                        if (rtc_command==0xB1) {
                        // clear FTU
                        if (rtc_value & 0x04) {
                            rtc_status=rtc_status&(~0x18);
                            int_status=int_status&(~0x04);
                        }
                    }       
                }
            }
        } else {
                        // else end or abort
                        phase=-1;
                        rtc_command=0;
                        rtc_value=0;
        }
        scr2 = val & 0xFFFF00FF;
        scr2 |= scr2_2<< 8; 
        old_scr2 = scr2_2;



/*  #define RTC_ADDRESS 0
    #define RTC_DATA    1
    #define SCR2_RTCLK 0x2
    #define SCR2_RTDATA 0x4

    static uint8_t rtc_status = 0x90;
    static uint8_t old_scr2;
    static int state = RTC_ADDRESS; 
    static int bits = 0;
    uint16_t ret;
    
    static uint8_t command = 0;
    static uint8_t data = 0;
    
    uint8_t scr2_2;
     
    if(size == 4)
        scr2_2 = (val >> 8) & 0xFF;
    else
        scr2_2 = val&0xFF;

    //DPRINTF("scr2 write %x 2_2: %x bits %x\n",val,scr2_2,bits);   
    time_t time_h = time(NULL);
    struct tm *info = localtime(&time_h);
  

    #define SCR2_TOBCD(x) (((x / 10) << 4) + ( x % 10)) 

    switch(command)
    {
        case 0x0 ... 0x1F:
            ret = rtc_ram[command];
            break;
        case 0x20:
            ret = SCR2_TOBCD(info->tm_sec);
            break;
        case 0x21:
            ret = SCR2_TOBCD(info->tm_min);
            break;
        case 0x22:
            ret = SCR2_TOBCD(info->tm_hour);
            break;
        case 0x23:
            ret = 0x00;
            break;
        case 0x24:
            ret = SCR2_TOBCD(info->tm_mday);
            break;
        case 0x25:
            ret = SCR2_TOBCD((info->tm_mon+1));
            break;
         case 0x26:
            ret = SCR2_TOBCD((info->tm_year - 100));
            break;
        case 0x30:
            ret = rtc_status;
            break;
        case 0x31:
            ret = 0;
            break;
        default:
            ret = 0x100;

    }



    
    switch(state)
    {
        case RTC_ADDRESS:
            if(((scr2_2 & 2) == 0) && (old_scr2 & 2))
            {
                command = (command << 1) | ((scr2_2 & SCR2_RTDATA) >> 2);

                bits++;
                if(bits == 8)
                {
                    //DPRINTF("RTC command %x\n",command);
                    bits = 0;
                    state = RTC_DATA;
                }
            }            
            break;
        case RTC_DATA:
            if(((scr2_2 & 2) == 0) && (old_scr2 & 2))
            {
               
                if(ret & 0x100)
                {
                    data = (data << 1) | ((scr2_2 & SCR2_RTDATA) >> 2);
                }
                else  
                    scr2_2 = scr2_2 & (~SCR2_RTDATA);
    
                if (ret & (0x80 >> (bits)))
                {
                    scr2_2 |= SCR2_RTDATA;
                }
                
                bits++;
                
                if(bits == 8)
                {
                    if(ret & 0x100)
                    {
                        if(command >= 0x80)
                        {
                            rtc_ram[command - 0x80] = data & 0xff;
                                //#ifdef READ_RTC
                            FILE *fp = fopen("rtc.ram", "wb+");
                            if(fwrite(rtc_ram, 1, 32, fp) != 32)
                                abort();
                            fclose(fp);
                            //#endif

                        } 
                        if (command == 0xB1) {
                            // clear FTU
                            if (data & 0x04) {
                                rtc_status = rtc_status & (~0x18);
                                int_status = int_status & (~0x04);
                            }
                        }   
                        
                    }
                    bits = 0;
                    state = RTC_ADDRESS;
                    command = 0;
                    data = 0;
                }
        }
        break;

    }

    scr2 = (0xFFFF00FF & scr2) | (scr2_2 << 8);
    old_scr2 = scr2_2;*/
}

static void mmio_writeb(void*opaque, target_phys_addr_t addr, uint32_t val)
{
    switch(addr)
    {
        case 0xd003:
            nextscr2_write(opaque, val, 1);
            break;
        default:
        DPRINTF("MMIO Write B @ %x with %x\n",addr,val);
    }

}
static void mmio_writew(void*opaque, target_phys_addr_t addr, uint32_t val)
{
    DPRINTF("MMIO Write W\n" );
}



static void mmio_writel(void*opaque, target_phys_addr_t addr, uint32_t val)
{
    #ifdef DEBUG_NEXT
    CPUState *s = (CPUState *)opaque;
    #endif
    
    switch(addr)
    {
        case 0x7000:
            DPRINTF("INT Status old: %x new: %x @ %x\n",int_status,val,s->pc);
            int_status = val;
            break;
        case 0x7800:
            DPRINTF("INT Mask old: %x new: %x @ %x\n",int_mask,val,s->pc);
            int_mask  = val;
            break;
        case 0xc000:
            DPRINTF("SCR1 Write: %x @ %X\n",val,((CPUM68KState *)opaque)->pc);
            break;
        case 0xd000:
            nextscr2_write(opaque, val, 4);
            break;
        
        default:
            DPRINTF("MMIO Write l @ %x with %x\n",addr,val);
            ;

    }
}



static uint32_t scr_readb(void*opaque, target_phys_addr_t addr)
{
    #ifdef DEBUG_NEXT
    CPUState *s = (CPUState *)opaque;
    #endif
    //static uint32_t eventc = 0; 
    switch(addr)
    {
        case 0x14108:
            DPRINTF("FD read @ %x %x\n",addr,s->pc);
            return 0x40 | 0x04 | 0x2 | 0x1;//2;
        case 0x14020:
            DPRINTF("SCSI 4020  STATUS READ %X\n",next_state.scsi_csr_1);
            return next_state.scsi_csr_1; 

        case 0x14021:
            DPRINTF("SCSI 4021 STATUS READ %X\n",next_state.scsi_csr_2);
            return 0x40; 
        
        /* these 4 registers are the hardware timer, not sure which register
         * is the latch instead of data, but no problems so far
         */ 
        case 0x1a000:
            return 0xff & (clock() >> 24);
        case 0x1a001: 
            return 0xff & (clock() >> 16);
        case 0x1a002: 
            return 0xff & (clock() >> 8);
        case 0x1a003: 
            //A hell of a hack, but all we need is to have this change
            //consistently, so this works for now
            //8/22/2011: mess uses almost the same thing apparetnly
            return 0xFF & clock(); 
        
        default:
            DPRINTF("BMAP Read B @ %x\n",addr);
            return 0;
    }
}
static uint32_t scr_readw(void*opaque, target_phys_addr_t addr)
{
    DPRINTF("BMAP Read W @ %x\n",addr);
    return 0;
}
static uint32_t scr_readl(void*opaque, target_phys_addr_t addr)
{
    DPRINTF("BMAP Read L @ %x\n",addr);
    return 0;
}
static void scr_writeb(void*opaque, target_phys_addr_t addr, uint32_t value)
{
    #ifdef DEBUG_NEXT
    CPUState *s = (CPUState *)opaque;
    #endif

    switch(addr)
    {
        case 0x14108:
            DPRINTF("FDCSR Write: %x @ %x\n",value,s->pc);
        
        if(value == 0x0)
        {
            
       //     qemu_irq_raise(next_state.fd_irq[0]);
        }
        break;
        case 0x14020://SCSI Control Register
            #define SCSICSR_ENABLE  0x01
            #define SCSICSR_RESET   0x02//reset scsi dma
            #define SCSICSR_FIFOFL  0x04
            #define SCSICSR_DMADIR  0x08//if set, scsi to mem
            #define SCSICSR_CPUDMA  0x10//if set, dma enabled
            #define SCSICSR_INTMASK 0x20//if set, interrupt enabled
            if(value & SCSICSR_FIFOFL)
            {
                DPRINTF("SCSICSR FIFO Flush\n");
                /* will have to add another irq to the esp if this is needed */
                //esp_puflush_fifo(esp_g);
                //qemu_irq_pulse(next_state.scsi_dma);
            }
            
            if(value & SCSICSR_ENABLE)
            {
                DPRINTF("SCSICSR Enable\n");
                //qemu_irq_raise(next_state.scsi_dma);
                //next_state.scsi_csr_1 = 0xc0;
                //next_state.scsi_csr_1 |= 0x1;
                    //qemu_irq_pulse(next_state.scsi_dma);
            }
           // else
                //next_state.scsi_csr_1 &= ~SCSICSR_ENABLE;
            
            if(value & SCSICSR_RESET)
            {
                DPRINTF("SCSICSR Reset\n");
                //i think this should set DMADIR. CPUDMA and INTMASK to 0 
                //abort();      
            
                //qemu_irq_raise(next_state.scsi_reset);
               // next_state.scsi_csr_1 &= (uint8_t)~(SCSICSR_INTMASK |0x80|0x1);
    
            }
            if(value & SCSICSR_DMADIR)
            {   
                DPRINTF("SCSICSR DMAdir\n");
            }
            if(value & SCSICSR_CPUDMA)
            {
                DPRINTF("SCSICSR CPUDMA\n");
                //qemu_irq_raise(next_state.scsi_dma);

                int_status |= 0x4000000;
            }else{
                int_status &= ~(0x4000000);


            }
            if(value & SCSICSR_INTMASK)
            {
                DPRINTF("SCSICSR INTMASK\n");
                //int_mask &= ~0x1000;
                //next_state.scsi_csr_1 |= value;
                //next_state.scsi_csr_1 &= ~SCSICSR_INTMASK;
             //   if(next_state.scsi_queued) {
                   // next_state.scsi_queued = 0;
                    //next_irq(opaque, NEXT_SCSI_I, level);

                
              //  }
            }else{

                //int_mask |= 0x1000;
            }
            if(value & 0x80)
            {

                //int_mask |= 0x1000;
                //next_state.scsi_csr_1 |= 0x80;

            }
            DPRINTF("SCSICSR Write: %x @ %x\n",value,s->pc);
           //     next_state.scsi_csr_1 = value;
            return;
        //Hardware timer latch - not implemented yet
        case 0x1a000:
        default:
            DPRINTF("BMAP Write B @ %x with %x\n",addr,value);
            
    }
}
static void scr_writew(void*opaque, target_phys_addr_t addr, uint32_t value)
{
    DPRINTF("BMAP Write W @ %x with %x\n",addr,value);
}
static void scr_writel(void*opaque, target_phys_addr_t addr, uint32_t value)
{
    DPRINTF("BMAP Write L @ %x with %x\n",addr,value);
}


static void next_cube_init(ram_addr_t ram_size,
                     const char *boot_device,
                     const char *kernel_filename, const char *kernel_cmdline,
                     const char *initrd_filename, const char *cpu_model)
{
    /* Initialize the cpu core */ 
    CPUState *env = cpu_init("m68040");
    //NextState *next_env = qemu_malloc(sizeof(NextState));
    //next_env->env = env;
    if (env == NULL) {
        fprintf(stderr, "Unable to find m68k CPU definition\n");
        exit(1);
    }
    
    /* Initialize CPU registers.  */
    env->vbr = 0;
    env->pc  = 0x100001e; //technically should read vector
    env->sr  = 0x2700;
    
    /* Set internal registers to initial values */
    /*     0x0000XX00 << vital bits */
    scr1 = 0x00011102;
    scr2 = 0x00ff0c80;
    
    int_mask = 0x0;//88027640; 
    int_status= 0x0;//200;
    
    /* Load RTC ram,  needs to be in a function probably */
    { 
        rtc_ram = qemu_malloc(32);
        //#ifdef LOAD_RTC
        FILE *fp = fopen("rtc.ram","rb");
        if(fp == NULL)
            memcpy(rtc_ram,rtc_ram2,32);
        else{
            if(fread(rtc_ram,1,32,fp) != 32)
                memcpy(rtc_ram,rtc_ram2,32);
            fclose(fp);
        }
        //#endif
        //the following line uses the built-in nvram value instead
       //memcpy(rtc_ram,rtc_ram2,32);
    }


    /* 64MB RAM starting at 0x4000000  */
    cpu_register_physical_memory(0x4000000, RAM_SIZE,
        qemu_ram_alloc(NULL, "next-cube.ram", RAM_SIZE) | IO_MEM_RAM);
  
    /* Framebuffer */
    nextfb_init(&nextfb_state);
 
    /* MMIO */
    cpu_register_physical_memory((uint32_t)0x2000000,0xD0000,
        cpu_register_io_memory(mmio_read, mmio_write, (void *)env,DEVICE_NATIVE_ENDIAN));    
    
    /* BMAP */ //acts as a catch-all for now
    cpu_register_physical_memory((uint32_t)0x2100000,0x3A7FF,
        cpu_register_io_memory(scr_read, scr_write, (void *)env,DEVICE_NATIVE_ENDIAN));

    /* KBD */
    nextkbd_init((void *)env);

    /* Serial */
    CharDriverState *console = serial_hds[0];//text_console_init(NULL);
    qemu_irq *serial = qemu_allocate_irqs(serial_irq, env, 2);
    escc_init(0x2118000, serial[0], serial[1], NULL, console, (9600*384), 0);
    
    /* Load ROM here */  
    if (bios_name == NULL) {
        bios_name = ROM_FILE;
    }
    char *bios_filename = qemu_find_file(QEMU_FILE_TYPE_BIOS, bios_name);
 
    if(get_image_size(bios_filename) != 0x20000)
    {
        fprintf(stderr,"Failed to load rom file!\n");
        exit(1);
    }
    
    /* still not sure if the rom should also be mapped at 0x0*/ 
    rom_add_file_fixed(bios_filename,0x1000000,0);
    cpu_register_physical_memory((uint32_t)0x1000000, 0x20000,
        qemu_ram_alloc(NULL, bios_filename, 0x20000) | IO_MEM_ROM);

    /* Ethernet */
    nextnet_init((void *)env);
    
    /* SCSI */
    //void *espdma;
    next_state.scsi_irq = qemu_allocate_irqs(nextscsi_irq, env, 1);
    qemu_irq *scsi = next_state.scsi_irq; 
    //qemu_irq esp_reset, dma_enable;
    esp_new_init(0x2114000, 0,
               nextscsi_read, nextscsi_write,
               (void *)env, scsi[0], &next_state.scsi_reset,
               &next_state.scsi_dma);
    
    //qdev_connect_gpio_out(espdma, 0, esp_reset);
    //qdev_connect_gpio_out(espdma, 1, dma_enable);

    /* FLOPPY */
    DriveInfo *fd[MAX_FD];
    /* FD terminal count, asserted when dma transfer is complete */
    //qemu_irq fdc_tc;
    //next_state.fd_irq = &fdc_tc;
    qemu_irq *fdc = qemu_allocate_irqs(nextfdc_irq, env, 3);
    int i;
    for(i = 0; i < MAX_FD; i++) {
        fd[i] = drive_get(IF_FLOPPY, 0, i);
    }
    //sun4m_fdctrl_init(fdc[0], 0x2114100, fd,
    //                 &fdc_tc);
  fdctrl_init_sysbus(fdc[0], 0,
                       0x2114100, fd);

    /* DMA */ 
    cpu_register_physical_memory((uint32_t)0x2000000,0x5000,
        cpu_register_io_memory(dma_read, dma_write, (void *)env,DEVICE_NATIVE_ENDIAN));   

}

void nextdma_write(void *opaque, uint8_t *buf, int size, int type);
void nextdma_write(void *opaque, uint8_t *buf, int size, int type)
{
    uint32_t base_addr;
    int irq = 0;
    uint8_t align = 16;
    if(type == NEXTDMA_ENRX || type == NEXTDMA_ENTX)
        align = 32;
    /* Most DMA is supposedly 16 byte aligned */    
    if((size % align) != 0)
    {
        size -= size % align;
        size += align;
    }
    
    /* prom sets the dma start using initbuf
        while the bootloader uses next so
        we check to see if initbuf is 0 */
    if(next_state.dma[type].next_initbuf == 0)
        base_addr = next_state.dma[type].next;
    else
        base_addr = next_state.dma[type].next_initbuf;
    
    cpu_physical_memory_write(base_addr,buf,size);
    
    next_state.dma[type].next_initbuf = 0;
    
    /* saved limit is checked to calculate packet size
        by both the rom and netbsd */ 
    next_state.dma[type].saved_limit = (next_state.dma[type].next + size);
    next_state.dma[type].saved_next  = (next_state.dma[type].next);
    
    /*32 bytes under savedbase seems to be some kind of register
    of which the purpose is unknown as of yet*/
    //stl_phys(s->rx_dma.base-32,0xFFFFFFFF);
    
    if(!(next_state.dma[type].csr & DMA_SUPDATE)){  
        next_state.dma[type].next  = next_state.dma[type].start;
        next_state.dma[type].limit = next_state.dma[type].stop;
    }

    //Set dma registers and raise an irq
    next_state.dma[type].csr |= DMA_COMPLETE; //DON'T CHANGE THIS!!!!
    
    switch(type)
    {
        case NEXTDMA_SCSI:
            irq = NEXT_SCSI_DMA_I;
            break; 
    }
    
    next_irq(opaque, irq, 1);
    next_irq(opaque, irq, 0);

}

void nextscsi_read(void *opaque, uint8_t *buf, int len)
{
    fprintf(stderr,"SCSI READ: %x\n",len);
    abort();
}

void nextscsi_write(void *opaque, uint8_t *buf, int size)
{
    DPRINTF("SCSI WRITE: %i\n",size);
    nextdma_write(opaque, buf, size, NEXTDMA_SCSI);
}

void nextfdc_irq(void *opaque, int n, int level)
{
    DPRINTF("FLOPPY IRQ LVL %i %i\n",n,level);
    level = !level;
    next_irq(opaque, NEXT_SCSI_I, level);
}

void nextscsi_irq(void *opaque, int n, int level)
{
    DPRINTF("SCSI IRQ NUM %i %i %x\n",n,level,int_mask);
    next_irq(opaque, NEXT_SCSI_I, level);
}
void serial_irq(void *opaque, int n, int level)
{
    DPRINTF("SCC IRQ NUM %i\n",n);
    if(n)
    next_irq(opaque, NEXT_SCC_DMA_I, level);
    else
    next_irq(opaque, NEXT_SCC_I, level);
}
#define NEXTDMA_SCSI(x)      0x10 + x
#define NEXTDMA_FD(x)        0x10 + x
#define NEXTDMA_ENTX(x)      0x110 + x
#define NEXTDMA_ENRX(x)      0x150 + x
#define NEXTDMA_CSR          0x0
#define NEXTDMA_SAVED_NEXT   0x3FF0 
#define NEXTDMA_SAVED_LIMIT  0x3FF4 
#define NEXTDMA_SAVED_START  0x3FF8 
#define NEXTDMA_SAVED_STOP   0x3FFc 
#define NEXTDMA_NEXT         0x4000 
#define NEXTDMA_LIMIT        0x4004 
#define NEXTDMA_START        0x4008 
#define NEXTDMA_STOP         0x400c 
#define NEXTDMA_NEXT_INIT    0x4200 
#define NEXTDMA_SIZE         0x4204 

static void dma_writel(void*opaque, target_phys_addr_t addr, uint32_t value)
{
    
    uint16_t reg = addr;
    //int channel = 0;
    switch(reg)
    {
    /*
        case SCSI"all registers:
        channel = NEXTDMA_SCSI;
        addr = addr - NEXTDMA_SCSI(0);



    */


    }
    
    switch(addr)
    {
        case NEXTDMA_ENRX(NEXTDMA_CSR):
                if(value & DMA_DEV2M)
                    next_state.dma[NEXTDMA_ENRX].csr |= DMA_DEV2M;
                
                if(value & DMA_SETENABLE)
                {
                    //DPRINTF("SCSI DMA ENABLE\n");
                    next_state.dma[NEXTDMA_ENRX].csr |= DMA_ENABLE;
                    //if(!(next_state.dma[NEXTDMA_ENRX].csr & DMA_DEV2M))
                        //fprintf(stderr,"DMA TO DEVICE\n");
                    //else 
                    //  DPRINTF("DMA TO CPU\n");
                    //if(next_state.scsi_csr_1 & 1<<3)
                    /// DPRINTF("SCSI DIR\n");
                }
                if(value & DMA_SETSUPDATE)
                    {
                        next_state.dma[NEXTDMA_ENRX].csr |= DMA_SUPDATE;   
                    }
                if(value & DMA_CLRCOMPLETE)
                    next_state.dma[NEXTDMA_ENRX].csr&= ~DMA_COMPLETE;

                if(value & DMA_RESET)
                {
                    next_state.dma[NEXTDMA_ENRX].csr &= ~(DMA_COMPLETE | DMA_SUPDATE | DMA_ENABLE | DMA_DEV2M);                
                }
              //  DPRINTF("RXCSR \tWrite: %x\n",value);
                break;
        case NEXTDMA_ENRX(NEXTDMA_NEXT_INIT):
            next_state.dma[NEXTDMA_ENRX].next_initbuf = value;
            //DPRINTF("DMA INITBUF WRITE\n");
            break; 
        case NEXTDMA_ENRX(NEXTDMA_NEXT):
            //DPRINTF("DMA NEXT WRITE\n");
            next_state.dma[NEXTDMA_ENRX].next = value;
            break;
        case NEXTDMA_ENRX(NEXTDMA_LIMIT):
            //DPRINTF("DMA NEXT WRITE\n");
            next_state.dma[NEXTDMA_ENRX].limit = value;
            break; 
        case NEXTDMA_SCSI(NEXTDMA_CSR):
                if(value & DMA_DEV2M)
                    next_state.dma[NEXTDMA_SCSI].csr |= DMA_DEV2M;
                
                if(value & DMA_SETENABLE)
                {
                    //DPRINTF("SCSI DMA ENABLE\n");
                    next_state.dma[NEXTDMA_SCSI].csr |= DMA_ENABLE;
                    //if(!(next_state.dma[NEXTDMA_SCSI].csr & DMA_DEV2M))
                        //fprintf(stderr,"DMA TO DEVICE\n");
                    //else 
                        //DPRINTF("DMA TO CPU\n");
                    //if(next_state.scsi_csr_1 & 1<<3)
                    /// DPRINTF("SCSI DIR\n");
                }
                if(value & DMA_SETSUPDATE)
                    {
                        next_state.dma[NEXTDMA_SCSI].csr |= DMA_SUPDATE;   
                        //DPRINTF("DMA SUPDATE\n");
                    }
                if(value & DMA_CLRCOMPLETE)
                    next_state.dma[NEXTDMA_SCSI].csr&= ~DMA_COMPLETE;

                if(value & DMA_RESET)
                {
                    next_state.dma[NEXTDMA_SCSI].csr &= ~(DMA_COMPLETE | DMA_SUPDATE | DMA_ENABLE | DMA_DEV2M);                
                
                    //DPRINTF("SCSI DMA RESET\n");
                }
              //  DPRINTF("RXCSR \tWrite: %x\n",value);
                break;
       
        case NEXTDMA_SCSI(NEXTDMA_NEXT):
            //DPRINTF("DMA NEXT WRITE\n");
            next_state.dma[NEXTDMA_SCSI].next = value;
            break; 
        
        case NEXTDMA_SCSI(NEXTDMA_LIMIT):
            next_state.dma[NEXTDMA_SCSI].limit = value;
            //DPRINTF("DMA LIMIT WRITE\n");
            break;

        case NEXTDMA_SCSI(NEXTDMA_START):
            next_state.dma[NEXTDMA_SCSI].start = value;
            //DPRINTF("DMA START WRITE\n");
            break;
    
        case NEXTDMA_SCSI(NEXTDMA_STOP):
            next_state.dma[NEXTDMA_SCSI].stop = value;
            //DPRINTF("DMA STOP WRITE\n");
            break;
        
        case NEXTDMA_SCSI(NEXTDMA_NEXT_INIT):
            next_state.dma[NEXTDMA_SCSI].next_initbuf = value;
            //DPRINTF("DMA INITBUF WRITE\n");
            break; 
 
        
        default:
            DPRINTF("DMA write @ %x w/ %x\n",addr, value);

    }    



}
static uint32_t dma_readl(void*opaque, target_phys_addr_t addr)
{
    switch(addr)
    {
        case NEXTDMA_SCSI(NEXTDMA_CSR):
            DPRINTF("SCSI DMA CSR READ\n");
            return next_state.dma[NEXTDMA_SCSI].csr;
        case NEXTDMA_ENRX(NEXTDMA_CSR):
            return next_state.dma[NEXTDMA_ENRX].csr;
        case NEXTDMA_ENRX(NEXTDMA_NEXT_INIT):
            return next_state.dma[NEXTDMA_ENRX].next_initbuf;
        case NEXTDMA_ENRX(NEXTDMA_NEXT):
            //DPRINTF("DMA NEXT READ\n");
            return next_state.dma[NEXTDMA_ENRX].next;
        case NEXTDMA_ENRX(NEXTDMA_LIMIT):
            //DPRINTF("DMA NEXT READ\n");
            vm_stop(VMSTOP_DEBUG);
            return next_state.dma[NEXTDMA_ENRX].limit;

        case NEXTDMA_SCSI(NEXTDMA_NEXT):
            //DPRINTF("DMA NEXT READ\n");
            return next_state.dma[NEXTDMA_SCSI].next;
        case NEXTDMA_SCSI(NEXTDMA_NEXT_INIT):
            return next_state.dma[NEXTDMA_SCSI].next_initbuf;
          case NEXTDMA_SCSI(NEXTDMA_LIMIT):
            return next_state.dma[NEXTDMA_SCSI].limit;
        case NEXTDMA_SCSI(NEXTDMA_START):
            return next_state.dma[NEXTDMA_SCSI].start;
 case NEXTDMA_SCSI(NEXTDMA_STOP):
            return next_state.dma[NEXTDMA_SCSI].stop;

        default:
        DPRINTF("DMA read @ %x\n",addr);
        return 0;

    }

    /* once the csr's are done, subtract 0x3FEC from the addr, and that will normalize the upper registers*/

}
void next_dma_write(void *opaque, const uint8_t *buf, int len, int type)
{




}

void next_dma_check(void)
{
    //loop through dma structs

    //



}
/* TODO: set the shift numbers as values in the enum, so the first switch
    will not be needed */
void next_irq(void *opaque, int number, int level)
{
    CPUM68KState *s = opaque;
    int shift = 0;
    /* first switch sets interupt status */
    //DPRINTF("IRQ %i\n",number);
    switch(number)
    {
        /* level 3 - floppy, kbd/mouse, power, 
        ether rx/tx, scsi, clock */
        case NEXT_FD_I:
            shift = 7;;
            break;
        case NEXT_KBD_I:
            shift = 3;
            break;
        case NEXT_PWR_I:
            shift = 2;
            break;
        case NEXT_ENRX_I:
            shift = 9;
            break;
        case NEXT_ENTX_I:
            shift = 10;
            break;
        case NEXT_SCSI_I:
            shift = 12;
            break;
        case NEXT_CLK_I:
            shift = 5;
            break;
 
        /* level 5 - scc (serial) */
        case NEXT_SCC_I:
            shift = 17;
            break;

        /* level 6 - audio etherrx/tx dma */
        case NEXT_ENTX_DMA_I:
            shift = 28;
            break;
        case NEXT_ENRX_DMA_I:
            shift = 27;
            break;
        case NEXT_SCSI_DMA_I:
            shift = 26;
            break;
        case NEXT_SND_I:
            shift = 23;
            break;
        case NEXT_SCC_DMA_I:
            shift = 21;
            break;

    }
    /* this HAS to be wrong, the interrupt handlers in mach and together  int_status and int_mask 
        and return if there is a hit */ 
    if(int_mask & (1 << shift)){
        DPRINTF("%x interrupt masked @ %x\n",1 << shift,s->pc);
        //return;
    }
    
    /* second switch triggers the correct interrupt */
    if(level){
        int_status |= 1 << shift;

        switch(number)
        {
            /* level 3 - floppy, kbd/mouse, power, 
            ether rx/tx, scsi, clock */
            case NEXT_FD_I:
            case NEXT_KBD_I:
            case NEXT_PWR_I:
            case NEXT_ENRX_I:
            case NEXT_ENTX_I:
            case NEXT_SCSI_I:
            case NEXT_CLK_I:
                m68k_set_irq_level(s,3,27);
                break; 
            
            /* level 5 - scc (serial) */
            case NEXT_SCC_I:
                m68k_set_irq_level(s,5,29);
                break;

            /* level 6 - audio etherrx/tx dma */
            case NEXT_ENTX_DMA_I:
            case NEXT_ENRX_DMA_I:
            case NEXT_SCSI_DMA_I:
            case NEXT_SND_I:
            case NEXT_SCC_DMA_I:
                m68k_set_irq_level(s,6,30);
                break;
        }
    
    }else{
        int_status &= ~(1 << shift);
        cpu_reset_interrupt(s, CPU_INTERRUPT_HARD); 
    }
}



static QEMUMachine next_machine = {
    .name = "next-cube",
    .desc = "NeXT Cube",
    .init = next_cube_init,
};

static void next_machine_init(void)
{
    qemu_register_machine(&next_machine);
}

machine_init(next_machine_init);
