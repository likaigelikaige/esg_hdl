
initial begin
  prm_rom[ADDR_TRIG_COUNT].prm         = "trate";
  prm_rom[ADDR_TRIG_COUNT].min         = 100;
  prm_rom[ADDR_TRIG_COUNT].max         = 100000;
  prm_rom[ADDR_TRIG_COUNT].units       = "pps";
  prm_rom[ADDR_TRIG_COUNT].rights      = r;
  prm_rom[ADDR_TRIG_COUNT].is_exec     = 0;
  //prm_ram[ADDR_TRIG_COUNT]             = 0;
      
  prm_rom[ADDR_PULSE_COUNT].prm        = "prate";
  prm_rom[ADDR_PULSE_COUNT].min        = 0;
  prm_rom[ADDR_PULSE_COUNT].max        = 100000;
  prm_rom[ADDR_PULSE_COUNT].units      = "pps";
  prm_rom[ADDR_PULSE_COUNT].rights     = r;
  prm_rom[ADDR_PULSE_COUNT].is_exec    = 0;
  // prm_ram[ADDR_PULSE_COUNT]             = 0;
     
  prm_rom[ADDR_CUR_TRIG].prm           = "cur_trg";
  prm_rom[ADDR_CUR_TRIG].min           = 0;
  prm_rom[ADDR_CUR_TRIG].max           = 2**12;
  prm_rom[ADDR_CUR_TRIG].units         = "units";
  prm_rom[ADDR_CUR_TRIG].rights        = r;
  prm_rom[ADDR_CUR_TRIG].is_exec       = 0;
  // prm_ram[ADDR_CUR_TRIG]            = 0;

  prm_rom[ADDR_AVERAGE_POWER].prm      = "avg_pwr";
  prm_rom[ADDR_AVERAGE_POWER].min      = 0;
  prm_rom[ADDR_AVERAGE_POWER].max      = 2**12;
  prm_rom[ADDR_AVERAGE_POWER].units    = "units";
  prm_rom[ADDR_AVERAGE_POWER].rights   = r;
  prm_rom[ADDR_AVERAGE_POWER].is_exec  = 0;

  prm_rom[ADDR_DECIMATION].prm         = "decim";
  prm_rom[ADDR_DECIMATION].min         = 0;
  prm_rom[ADDR_DECIMATION].max         = 30000;
  prm_rom[ADDR_DECIMATION].units       = "units";
  prm_rom[ADDR_DECIMATION].rights      = rw;
  prm_rom[ADDR_DECIMATION].is_exec     = 0;
  // prm_ram[ADDR_DECIMATION]            = 15000;
     
  prm_rom[ADDR_DETECT_OFFSET].prm      = "det_os";
  prm_rom[ADDR_DETECT_OFFSET].min      = 0;
  prm_rom[ADDR_DETECT_OFFSET].max      = 2**12;
  prm_rom[ADDR_DETECT_OFFSET].units    = "units";
  prm_rom[ADDR_DETECT_OFFSET].rights   = rw;
  prm_rom[ADDR_DETECT_OFFSET].is_exec  = 0;
  // prm_ram[ADDR_DETECT_OFFSET]            = 0;
     
  prm_rom[ADDR_TRIG_OFFSET].prm        = "trg_os";
  prm_rom[ADDR_TRIG_OFFSET].min        = 0;
  prm_rom[ADDR_TRIG_OFFSET].max        = 2**12;
  prm_rom[ADDR_TRIG_OFFSET].units      = "units";
  prm_rom[ADDR_TRIG_OFFSET].rights     = rw;
  prm_rom[ADDR_TRIG_OFFSET].is_exec    = 0;

  prm_rom[ADDR_LOW_TRIG_TICKS].prm     = "low_tks";
  prm_rom[ADDR_LOW_TRIG_TICKS].min     = 0;
  prm_rom[ADDR_LOW_TRIG_TICKS].max     = 2000;
  prm_rom[ADDR_LOW_TRIG_TICKS].units   = "ticks";
  prm_rom[ADDR_LOW_TRIG_TICKS].rights  = rw;
  prm_rom[ADDR_LOW_TRIG_TICKS].is_exec = 0;
  // execultable parameters
  prm_rom[ADDR_ENABLE].prm     = "enable";
  prm_rom[ADDR_ENABLE].min     = 0;
  prm_rom[ADDR_ENABLE].max     = 1; // 0 = disable, 1 = enable
  prm_rom[ADDR_ENABLE].units   = "";
  prm_rom[ADDR_ENABLE].rights  = r;
  prm_rom[ADDR_ENABLE].is_exec = 1;
//  prm_ram[6]         = 0;
end