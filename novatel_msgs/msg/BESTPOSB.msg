# message 42
novatel_msgs/CommonHeader header

uint8 NONE=0        
uint8 FIXED=1       
uint8 FIXEDHEIGHT=2 
uint8 FLOATCONV=4        
uint8 WIDELANE=5         
uint8 NARROWLANE=6       
uint8 DOPPLER_VELOCITY=8 
uint8 SINGLE=16          
uint8 PSRDIFF=17         
uint8 WAAS=18            
uint8 PROPOGATED=19      
uint8 OMNISTAR=20        
uint8 L1_FLOAT=32        
uint8 IONOFREE_FLOAT=33  
uint8 NARROW_FLOAT=34    
uint8 L1_INT=48          
uint8 WIDE_INT=49        
uint8 NARROW_INT=50      
uint8 RTK_DIRECT_INS=51  
uint8 INS_SBAS=52        
uint8 INS_PSRSP=53       
uint8 INS_PSRDIFF=54     
uint8 INS_RTKFLOAT=55    
uint8 INS_RTKFIXED=56    

uint32 solution_status
uint32 pos_type

float64 latitude
float64 longitude
float64 altitude

float32 und
uint32 datum_id

float32 lat_std
float32 lon_std
float32 hgt_std

char[4] stn_id

float32 diff_age
float32 sol_age

uint8 svs
uint8 sol_svs
uint8 l1_svs
uint8 mult_svs
uint8 reserved
uint8 extsolstat
uint8 gal_sig_mask
uint8 sig_mask

