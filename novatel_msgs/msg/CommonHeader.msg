string START_MESSAGE=170

uint8 start
uint8 sync2
uint8 sync3

uint8 hdr_length   # header length
uint16 id #=93
uint8 msg_type

uint8 port_addr
uint8 length       #=72
uint8 sequence     # ushort

uint16 idle_time   # float
uint16 time_stat   # enum

uint16 week        # ushort
int32 gpssec       # GPSsec

uint32 rcvrstat    # ulong
uint16 reserved_h  # ushort
uint16 version_sw  # ushort 






