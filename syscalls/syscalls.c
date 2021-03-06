/**************************************************************************//*****
 * @file     stdio.c
 * @brief    Implementation of newlib syscall
 ********************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>

#undef errno
extern int errno;
extern int  _end;

__attribute__ ((used))
caddr_t _sbrk ( int incr )
{
  static unsigned char *heap = NULL;
  unsigned char *prev_heap;

  if (heap == NULL) {
    heap = (unsigned char *)&_end;
  }
  prev_heap = heap;

  heap += incr;

  return (caddr_t) prev_heap;
}

__attribute__ ((used))
int link(char *old, char *new) {
return -1;
}

__attribute__ ((used))
int _close(int file)
{
  return -1;
}

__attribute__ ((used))
int _fstat(int file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

__attribute__ ((used))
int _isatty(int file)
{
  return 1;
}

__attribute__ ((used))
int _lseek(int file, int ptr, int dir)
{
  return 0;
}

/*
__attribute__ ((used))
int _read(int file, char *ptr, int len)
{
  return 0;
}
__attribute__ ((used))
int _write(int file, char *ptr, int len)
{
  return len;
}
*/

int _read(int file, char *ptr, int len)
{
 int c, rxCount = 0;
 (void) file;
 while (len--) {
  if ((c = SH_GetChar()) != -1) {
// Getchar Function need to be replaced by the function you used to get values.
   *ptr++ = c;
   rxCount++;
  } else {
   break;
  }
 }
 if (rxCount <= 0) {
  return -1; /* Error exit */
 }
 return rxCount;
}

int _write(int file, char *ptr, int len)
{
int txCount;
 (void) file;
 for (txCount = 0; txCount < len; txCount++) {
  SH_SendChar(*ptr++);
// SendChar Function need to be replaced by the function you used to print values.
 }
 return len;
}

__attribute__ ((used))
void abort(void)
{
  /* Abort called */
  while(1);
}
          
/* --------------------------------- End Of File ------------------------------ */
