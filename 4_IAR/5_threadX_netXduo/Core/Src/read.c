/*******************
 *
 * Copyright 1998-2010 IAR Systems AB.  
 *
 * This is a template implementation of the "__read" function used by
 * the standard library.  Replace it with a system-specific
 * implementation.
 *
 * The "__read" function reads a number of bytes, at most "size" into
 * the memory area pointed to by "buffer".  It returns the number of
 * bytes read, 0 at the end of the file, or _LLIO_ERROR if failure
 * occurs.
 *
 * The template implementation below assumes that the application
 * provides the function "MyLowLevelGetchar".  It should return a
 * character value, or -1 on failure.
 *
 ********************/

#ifdef __IAR_SYSTEMS_ICC__

#include <yfuns.h>

#pragma module_name = "?__read"

int MyLowLevelGetchar();

size_t __read(int handle, unsigned char *buffer, size_t size)
{
  int nChars = 0;

  // This template only reads from "standard in", for all other file handles it returns failure.
  if (handle != _LLIO_STDIN)
  {
    return _LLIO_ERROR;
  }

  for (/* Empty */; size > 0; --size)
  {
    int c = MyLowLevelGetchar();
    if(c < 0)
      break;

    *buffer++ = c;
    ++nChars;
  }

  return nChars;
}

#endif
