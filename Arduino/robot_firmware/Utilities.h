/*
 * This file is part of robot_firmware
 * Copyright (C) 2018 Andrew Miyaguchi <andrewmiyaguchi@gmail.com>
 */

#ifndef UTILITIES_H
#define UTILITIES_H

/* printf support */
inline void pf(const char *fmt, ... ){
  char *buf = new char[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, 128, fmt, args);
  va_end (args);
  Serial.print(buf);
  delete[] buf;
}

/* printf using progmem */
inline void pf(const __FlashStringHelper *fmt, ... ){
  char *buf = new char[128];
  va_list args;
  va_start (args, fmt);
  vsnprintf_P(buf, sizeof(buf), (const char *)fmt, args);
  va_end(args);
  Serial.print(buf);
  delete[] buf;
}

/* Prints a debug message with timestamp in seconds */
inline void dmesg(const char *fmt, ... ) {
  float timeStamp;
  char *timeStr = new char[12];
  char *buf = new char[128];
  
  /* Apply arguments to the buffer */
  va_list args;
  va_start (args, fmt );
  vsnprintf(buf, 128, fmt, args);
  va_end (args);

  /* Apply time stamp */
  timeStamp = millis()/1000.0;
  dtostrf(timeStamp,11,3, timeStr);

  /* Formatted print */
  pf("[%s] %s", timeStr, buf);

  /* Clenup */
  delete[] timeStr;
  delete[] buf;
}

/* debug message with a string programmed with F() Macro */
inline void dmesg(const __FlashStringHelper *strP) {
  size_t strLen = strlen_P((const char*) strP);
  char* str = new char[strLen + 1];
  strcpy_P(str, (const char *) strP);
  dmesg((const char *) str);
  delete[] str;
}

inline float fmap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif /* UTILITIES_H */
