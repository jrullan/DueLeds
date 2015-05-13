

__attribute__( ( always_inline ) ) static __INLINE uint32_t __ROR(uint32_t value, uint32_t shift)
{
  uint32_t result;
  
  __ASM volatile ("ror %0, %1, %2" : "=r" (result) : "r" (value) , "r" (shift) );
  return(result);
}

__attribute__( ( always_inline ) ) static __INLINE uint32_t __LSR(uint32_t value, uint32_t shift)
{
  uint32_t result;
  
  __ASM volatile ("lsr %0, %1, %2" : "=r" (result) : "r" (value) , "r" (shift) );
  return(result);
}
/*
__attribute__( ( always_inline ) ) static __INLINE uint32_t __RBIT(uint32_t value)
{
  uint32_t result;
  
  __ASM volatile ("rbit %0, %1" : "=r" (result) : "r" (value));
  return(result);
}
*/

  //Build dataByte[array]
  /*
  for(i=numBytes;i!=0;i--){
		pos = numBytes - i;
		dataBytes[pos] = 
			(ptr[pos] & (i+1)) +
			(ptr[pos + numBytes] & 0x02) +
			(ptr[pos + numBytes * 2] & 0x04) +
			(ptr[pos + numBytes * 3] & 0x08) +
			(ptr[pos + numBytes * 4] & 0x10) +
			(ptr[pos + numBytes * 5] & 0x20) +
			(ptr[pos + numBytes * 6] & 0x40) +
			(ptr[pos + numBytes * 7] & 0x80);
	}
	* 
	*/
