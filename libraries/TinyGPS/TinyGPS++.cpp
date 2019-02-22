/*
TinyGPS++ - a small GPS library for Arduino providing universal NMEA parsing
Based on work by and "distanceBetween" and "courseTo" courtesy of Maarten Lamers.
Suggestion to add satellites, courseTo(), and cardinal() by Matt Monson.
Location precision improvements suggested by Wayne Holder.
Copyright (C) 2008-2013 Mikal Hart
All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "TinyGPS++.h"

#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>

#define _GPGGAterm   "GPGGA"
#define _GPVTGterm   "GPVTG"
#define _PHROTterm   "PHROT"
#define _PHSPDterm   "PHSPD"
#define _PHTRHterm   "PHTRH"
#define _PHTROterm   "PHTRO"
#define _HEHDTterm   "HEHDT"
#define _HEROTterm   "HEROT"
#define _HETHSterm   "HETHS"

TinyGPSPlus::TinyGPSPlus()
  :  parity(0)
  ,  isChecksumTerm(false)
  ,  curSentenceType(GPS_SENTENCE_OTHER)
  ,  curTermNumber(0)
  ,  curTermOffset(0)
  ,  sentenceHasFix(false)
  ,  customElts(0)
  ,  customCandidates(0)
  ,  encodedCharCount(0)
  ,  sentencesWithFixCount(0)
  ,  failedChecksumCount(0)
  ,  passedChecksumCount(0)
{
  term[0] = '\0';
}

//
// public methods
//

bool TinyGPSPlus::encode(char c)
{
  ++encodedCharCount;

  switch(c)
  {
  case ',': // term terminators
    parity ^= (uint8_t)c;
  case '\r':
  case '\n':
  case '*':
    {
      bool isValidSentence = false;
      if (curTermOffset < sizeof(term))
      {
        term[curTermOffset] = 0;
        isValidSentence = endOfTermHandler();
      }
      ++curTermNumber;
      curTermOffset = 0;
      isChecksumTerm = c == '*';
      return isValidSentence;
    }
    break;

  case '$': // sentence begin
    curTermNumber = curTermOffset = 0;
    parity = 0;
    curSentenceType = GPS_SENTENCE_OTHER;
    isChecksumTerm = false;
    sentenceHasFix = false;
    return false;

  default: // ordinary characters
    if (curTermOffset < sizeof(term) - 1)
      term[curTermOffset++] = c;
    if (!isChecksumTerm)
      parity ^= c;
    return false;
  }

  return false;
}

//
// internal utilities
//
int TinyGPSPlus::fromHex(char a)
{
  if (a >= 'A' && a <= 'F')
    return a - 'A' + 10;
  else if (a >= 'a' && a <= 'f')
    return a - 'a' + 10;
  else
    return a - '0';
}

// static
// Parse a (potentially negative) number with up to 2 decimal digits -xxxx.yy
int32_t TinyGPSPlus::parseDecimal(const char *term)
{
  bool negative = *term == '-';
  if (negative) ++term;
  int32_t ret = 100 * (int32_t)atol(term);
  while (isdigit(*term)) ++term;
  if (*term == '.' && isdigit(term[1]))
  {
    ret += 10 * (term[1] - '0');
    if (isdigit(term[2]))
      ret += term[2] - '0';
  }
  return negative ? -ret : ret;
}

// static
// Parse degrees in that funny NMEA format DDMM.MMMM
void TinyGPSPlus::parseDegrees(const char *term, RawDegrees &deg)
{
  uint32_t leftOfDecimal = (uint32_t)atol(term);
  uint16_t minutes = (uint16_t)(leftOfDecimal % 100);
  uint32_t multiplier = 10000000UL;
  uint32_t tenMillionthsOfMinutes = minutes * multiplier;

  deg.deg = (int16_t)(leftOfDecimal / 100);

  while (isdigit(*term))
    ++term;

  if (*term == '.')
    while (isdigit(*++term))
    {
      multiplier /= 10;
      tenMillionthsOfMinutes += (*term - '0') * multiplier;
    }

  deg.billionths = (5 * tenMillionthsOfMinutes + 1) / 3;
  deg.negative = false;
}

#define COMBINE(sentence_type, term_number) (((unsigned)(sentence_type) << 5) | term_number)

// Processes a just-completed term
// Returns true if new sentence has just passed checksum test and is validated
bool TinyGPSPlus::endOfTermHandler()
{
  // If it's the checksum term, and the checksum checks out, commit
  if (isChecksumTerm)
  {
    char checksum = 16 * fromHex(term[0]) + fromHex(term[1]);
    if (checksum == parity)
    {
      passedChecksumCount++;
      if (sentenceHasFix)
        ++sentencesWithFixCount;

      switch(curSentenceType)
      {
      case GPS_SENTENCE_GPGGA:
        time.commit();
        if (sentenceHasFix)
        {
          location.commit();
          altitude.commit();
        }
        satellites.commit();
        hdop.commit();
        break;
      
      case GPS_SENTENCE_GPVTG:
    	gpvtg_course.commit();
    	gpvtg_speed.commit();
        break;
    
	  case GPS_SENTENCE_PHROT:
		phrot_roll.commit();
		phrot_pitch.commit();
		phrot_heading.commit();
		break;
  
      case GPS_SENTENCE_PHSPD:
    	phspd_surge.commit();
    	phspd_sway.commit();
    	phspd_heave.commit();
	    break;

	  case GPS_SENTENCE_PHTRH:
		phtrh_pitch.commit();
		phtrh_roll.commit();
		break;
	  
	  case GPS_SENTENCE_PHTRO:
		phtro_pitch.commit();
		phtro_roll.commit();
		break;
	  
	  case GPS_SENTENCE_HEHDT:
		hehdt_heading.commit();
		break;
	  
	  case GPS_SENTENCE_HEROT:
		herot_heading.commit();
		break;
	  
	  case GPS_SENTENCE_HETHS:
		heths_heading.commit();
		break;
	}

      return true;
    }

    else
    {
      ++failedChecksumCount;
    }

    return false;
  }

  // the first term determines the sentence type
  if (curTermNumber == 0)
  {
    if (!strcmp(term, _GPGGAterm))
      curSentenceType = GPS_SENTENCE_GPGGA;
    
    else if (!strcmp(term, _GPVTGterm))
      curSentenceType = GPS_SENTENCE_GPVTG;
    
    else if (!strcmp(term, _PHROTterm))
      curSentenceType = GPS_SENTENCE_PHROT;
    
    else if (!strcmp(term, _PHSPDterm))
      curSentenceType = GPS_SENTENCE_PHSPD;
    
    else if (!strcmp(term, _PHTRHterm))
      curSentenceType = GPS_SENTENCE_PHTRH;
    
    else if (!strcmp(term, _PHTROterm))
      curSentenceType = GPS_SENTENCE_PHTRO;
    
    else if (!strcmp(term, _HEHDTterm))
      curSentenceType = GPS_SENTENCE_HEHDT;
    
    else if (!strcmp(term, _HEROTterm))
      curSentenceType = GPS_SENTENCE_HEROT;
    
    else if (!strcmp(term, _HETHSterm))
      curSentenceType = GPS_SENTENCE_HETHS;
    
    else
      curSentenceType = GPS_SENTENCE_OTHER;
    return false;
  }

  if (curSentenceType != GPS_SENTENCE_OTHER && term[0])
    switch(COMBINE(curSentenceType, curTermNumber))
  {
    case COMBINE(GPS_SENTENCE_GPGGA, 1):
      time.setTime(term);
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 2):
      location.setLatitude(term);
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 3):
      location.rawNewLatData.negative = term[0] == 'S';
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 4):
      location.setLongitude(term);
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 5):
      location.rawNewLngData.negative = term[0] == 'W';
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 6): // Fix data (GPGGA)
      sentenceHasFix = term[0] > '0';
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 7): // Satellites used (GPGGA)
      satellites.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 8): // HDOP
      hdop.set(term);
      break;
    case COMBINE(GPS_SENTENCE_GPGGA, 9): // Altitude (GPGGA)
      altitude.set(term);
      break;
      
    case COMBINE(GPS_SENTENCE_GPVTG, 3):
	  gpvtg_course.set(term);
	  break;
	  
    case COMBINE(GPS_SENTENCE_GPVTG, 7):
	  gpvtg_speed.set(term);
	  break;
	  
    case COMBINE(GPS_SENTENCE_PHROT, 1):
	  phrot_roll.set(term);
	  break;
	  
    case COMBINE(GPS_SENTENCE_PHROT, 2):
	  phrot_pitch.set(term);
	  break;
	  
    case COMBINE(GPS_SENTENCE_PHROT, 3):
	  phrot_heading.set(term);
	  break;
	  
    case COMBINE(GPS_SENTENCE_PHSPD, 1):
	  phspd_surge.set(term);
	  break;
	  
    case COMBINE(GPS_SENTENCE_PHSPD, 2):
	  phspd_sway.set(term);
	  break;
	  
    case COMBINE(GPS_SENTENCE_PHSPD, 3):
	  phspd_heave.set(term);
	  break;
	  
    case COMBINE(GPS_SENTENCE_PHTRH, 1):
	  phtrh_pitch.set(term);
	  break;
	  
    case COMBINE(GPS_SENTENCE_PHTRH, 3):
	  phtrh_roll.set(term);
	  break;
	  
    case COMBINE(GPS_SENTENCE_PHTRO, 1):
	  phtro_pitch.set(term);
	  break;
	  
    case COMBINE(GPS_SENTENCE_PHTRO, 3):
	  phtro_roll.set(term);
	  break;
	  
    case COMBINE(GPS_SENTENCE_HEHDT, 1):
	  hehdt_heading.set(term);
	  break;
	  
    case COMBINE(GPS_SENTENCE_HEROT, 1):
	  herot_heading.set(term);
	  break;
    	  
    case COMBINE(GPS_SENTENCE_HETHS, 1):
	  heths_heading.set(term);
	  break;
  }

  return false;
}

const char *TinyGPSPlus::cardinal(double course)
{
  static const char* directions[] = {"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"};
  int direction = (int)((course + 11.25f) / 22.5f);
  return directions[direction % 16];
}

void TinyGPSLocation::commit()
{
   rawLatData = rawNewLatData;
   rawLngData = rawNewLngData;
   lastCommitTime = AP_HAL::millis();
   valid = updated = true;
}

void TinyGPSLocation::setLatitude(const char *term)
{
   TinyGPSPlus::parseDegrees(term, rawNewLatData);
}

void TinyGPSLocation::setLongitude(const char *term)
{
   TinyGPSPlus::parseDegrees(term, rawNewLngData);
}

double TinyGPSLocation::lat()
{
   updated = false;
   double ret = rawLatData.deg + rawLatData.billionths / 1000000000.0;
   return rawLatData.negative ? -ret : ret;
}

double TinyGPSLocation::lng()
{
   updated = false;
   double ret = rawLngData.deg + rawLngData.billionths / 1000000000.0;
   return rawLngData.negative ? -ret : ret;
}

void TinyGPSDate::commit()
{
   date = newDate;
   lastCommitTime = AP_HAL::millis();
   valid = updated = true;
}

void TinyGPSTime::commit()
{
   time = newTime;
   lastCommitTime = AP_HAL::millis();
   valid = updated = true;
}

void TinyGPSTime::setTime(const char *term)
{
   newTime = (uint32_t)TinyGPSPlus::parseDecimal(term);
}

void TinyGPSDate::setDate(const char *term)
{
   newDate = atol(term);
}

uint16_t TinyGPSDate::year()
{
   updated = false;
   uint16_t year_ = date % 100;
   return year_ + 2000;
}

uint8_t TinyGPSDate::month()
{
   updated = false;
   return (date / 100) % 100;
}

uint8_t TinyGPSDate::day()
{
   updated = false;
   return date / 10000;
}

uint8_t TinyGPSTime::hour()
{
   updated = false;
   return time / 1000000;
}

uint8_t TinyGPSTime::minute()
{
   updated = false;
   return (time / 10000) % 100;
}

uint8_t TinyGPSTime::second()
{
   updated = false;
   return (time / 100) % 100;
}

uint8_t TinyGPSTime::centisecond()
{
   updated = false;
   return time % 100;
}

void TinyGPSDecimal::commit()
{
   val = newval;
   lastCommitTime = AP_HAL::millis();
   valid = updated = true;
}

void TinyGPSDecimal::set(const char *term)
{
   newval = TinyGPSPlus::parseDecimal(term);
}

void TinyGPSInteger::commit()
{
   val = newval;
   lastCommitTime = AP_HAL::millis();
   valid = updated = true;
}

void TinyGPSInteger::set(const char *term)
{
   newval = atol(term);
}

TinyGPSCustom::TinyGPSCustom(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber)
{
   begin(gps, _sentenceName, _termNumber);
}

void TinyGPSCustom::begin(TinyGPSPlus &gps, const char *_sentenceName, int _termNumber)
{
   lastCommitTime = 0;
   updated = valid = false;
   sentenceName = _sentenceName;
   termNumber = _termNumber;
   memset(stagingBuffer, '\0', sizeof(stagingBuffer));
   memset(buffer, '\0', sizeof(buffer));

   // Insert this item into the GPS tree
   gps.insertCustom(this, _sentenceName, _termNumber);
}

void TinyGPSCustom::commit()
{
   strcpy(this->buffer, this->stagingBuffer);
   lastCommitTime = AP_HAL::millis();
   valid = updated = true;
}

void TinyGPSCustom::set(const char *term)
{
   strncpy(this->stagingBuffer, term, sizeof(this->stagingBuffer));
}

void TinyGPSPlus::insertCustom(TinyGPSCustom *pElt, const char *sentenceName, int termNumber)
{
   TinyGPSCustom **ppelt;

   for (ppelt = &this->customElts; *ppelt != NULL; ppelt = &(*ppelt)->next)
   {
      int cmp = strcmp(sentenceName, (*ppelt)->sentenceName);
      if (cmp < 0 || (cmp == 0 && termNumber < (*ppelt)->termNumber))
         break;
   }

   pElt->next = *ppelt;
   *ppelt = pElt;
}
