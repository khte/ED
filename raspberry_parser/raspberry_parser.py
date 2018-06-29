"""
uBlox UBX protocol spoofer for AutoQuad

Author: Kristian Husum Terkildsen, khte@mmmi.sdu.dk, November '17

"""

import serial
import struct
import time
import datetime
from src.utm import utmconv

class NMEAReader():
	def __init__(self):
		self.serialIn = serial.Serial(port = "/dev/ttyUSB0", baudrate = 115200)
		self.posFound = False
		self.uc = utmconv()

	def getPos(self):
		self.posFound = False
		while not self.posFound:
			readLine = self.serialIn.readline()
			if "GGA" in readLine:
				data = readLine.split(",")
				if len(data) > 5:
					try:
						lat = float(data[3]) / 100
						lon = float(data[4]) / 100
						alt = float(data[5])
					
						return lat, lon, alt

					except ValueError:
						pass  # do nothing!

class UBXSpoofer():
	def __init__(self):
		self.serialOut = serial.Serial(port = "/dev/ttyUSB1")
		#UBX header and ID's
		self.SYNC = b"\xb5\x62"
		self.NAV_VELNED = b"\x01\x12"
		self.NAV_POSLLH = b"\x01\x02"
		self.NAV_DOP = b"\x01\x04"
		self.NAV_TIMEUTC = b"\x01\x21"
		#Other variables
		self.messageClass = ""
		self.messageID = ""
		self.baudFound = False
		self.debug = True
		self.acc = 500
		#Instantiation of other classes
		self.r = NMEAReader()
	
	def startSpoofing(self):
		#Find the requested baud rate

		while not self.baudFound:
			readByte = self.serialOut.read()
			if readByte == "\xb5":
				readBytes = self.serialOut.read(3)
				if readBytes[1] == "\x06" and readBytes[2] == "\x00":
					readBytes = self.serialOut.read(14)
					foundBaudrate = ord(readBytes[10]) + ord(readBytes[11]) * 256 + ord(readBytes[12]) * 65536 + ord(readBytes[13]) * 16777216
					print "Baudrate: ", foundBaudrate
					self.serialOut.baudrate = foundBaudrate
					self.baudFound = True
					if self.debug == True:
						print "CFG-PRT message received, port configured"

		#Send positional data to flight controller
		print "Parsing positional data"
		GPSMsToW = 0
		while True:
			### read positional data from source
			lat, lon, alt = self.r.getPos()
			
			self.sendNAV_POSLLH(GPSMsToW, lat, lon, 0, alt, self.acc, self.acc)
			self.sendNAV_DOP(GPSMsToW, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0)

	def checksum(self, message):
		CK_A = 0x00
		CK_B = 0x00
		buf = buffer(message)
		for i in buf:
			CK_A += ord(i) #Return integer value representing the character
			CK_A &= 255
			CK_B += CK_A
			CK_B &= 255
		return struct.pack('BB', CK_A, CK_B)
	
	"""
	def calcMsToW(self):
		weekday = datetime.datetime.today().weekday()
		time = datetime.datetime.now()
		GPSMsToW = weekday * 86400000 + time.hour * 3600000 + time.minute * 60000 + time.second * 1000
		return GPSMsToW		
	"""
	"""Input units: ms, cm/s, cm/s, cm/s, cm/s, cm/s, deg, cm/s, deg"""
	"""
	def sendNAV_VELNED(self, GPSMsToW, velN, velE, velD, speed, gSpeed, heading, sAcc, headingAcc):
		msg = struct.pack('<cccchLlllLLlLL', self.SYNC[0], self.SYNC[1], self.NAV_VELNED[0], self.NAV_VELNED[1], 36, GPSMsToW, velN, velE, velD, speed, gSpeed, heading, sAcc, cAcc)
		msg += self.checksum(msg[2:])
		self.serialOut.write(msg)
	"""
	"""Input units: ms, deg, deg, mm, mm, mm, mm"""
	def sendNAV_POSLLH(self, GPSMsToW, lat, lon, height, hMSL, hAcc, vAcc):
		msg = struct.pack('<cccchLllllLL', self.SYNC[0], self.SYNC[1], self.NAV_POSLLH[0], self.NAV_POSLLH[1], 28, GPSMsToW, lon * 10000000, lat * 10000000, height, hMSL, hAcc, vAcc)
		msg += self.checksum(msg[2:])
		self.serialOut.write(msg)
	
	"""Input units: ms"""
	def sendNAV_DOP(self, GPSMsToW, gDOP, pDOP, tDOP, vDOP, hDOP, nDOP, eDOP):
		msg = struct.pack('<cccchLHHHHHHH', self.SYNC[0], self.SYNC[1], self.NAV_DOP[0], self.NAV_DOP[1], 18, GPSMsToW, gDOP * 100, pDOP * 100, tDOP * 100, vDOP * 100, hDOP * 100, nDOP * 100, eDOP * 100)
		msg += self.checksum(msg[2:])
		self.serialOut.write(msg)
		
	"""Input units: ns, ns, year, month, day, hour, minute, second"""
	"""
	def sendNAV_TIMEUTC(self, GPSMsToW, tAcc, nano, year, month, day, hour, minute, sec, valid):
		msg = struct.pack('<cccchLLlHhhhhhc', self.SYNC[0], self.SYNC[1], self.NAV_TIMEUTC[0], self.NAV_TIMEUTC[1], 20, GPSMsToW, tAcc, nano, year, month, day, hour, minute, sec, valid)
		msg += self.checksum(msg[2:])
		self.serialOut.write(msg)
	"""

spoof = UBXSpoofer()
spoof.startSpoofing()
