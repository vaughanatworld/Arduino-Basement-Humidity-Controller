ArduinoBasementHumididtyController
==================================

Arduino baseement humidity controller. 
Adaptation of the 2010 Wiznet Circuit Cellar  David Penrose design.

Humidity control is essential in residential and industrial buildings alike. 
This handy humidity control system calculates water vapor pressure from 
temperature and humidity readings. When the design detects that the outside 
air is drier than the air indoors, it triggers a ventilation system as 
opposed to a dehumidifier. A W7100 enables a user to monitor and control 
the moisture removal process via any PC with a standard Web browser. 
File data is stored on a memory stick so it can be transferred easily 
to a PC. (David Penrose)

My adaptation runs on an arduino UNO R3 with an ethernet shield with a W5100.
The original design was written in 8051 assembler. This design uses C and as
many generic Arduino libraries as possible.

The starting project is Daniel McClain's "Arduino Temp / Humidity Monitor with Web and SNMP"
project page: http://makeprojects.com/Project/Arduino+Temp+-+Humidity+Monitor+with+Web+and+SNMP/1367/1#.USJs52et1ts
source code: http://make-documents.s3.amazonaws.com/4LOqMXuaviAvi1ta.pdf