import RPi.GPIO as GPIO
import time
import drivers
import math
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008
import asyncio
import numpy as np
import haversine as hs
from mavsdk import System
from mavsdk.offboard import (OffboardError, PositionNedYaw)




GPIO.setmode(GPIO.BCM) # Broadcom pin-numbering scheme

GPIO.setup(4, GPIO.OUT) # output rf
GPIO.setup(27,GPIO.IN)
GPIO.setup(22,GPIO.IN)

# Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

display = drivers.Lcd()

system_addr = "serial:///dev/ttyUSB0"
drone = TakeoffDrone()

# Initial state for LEDs:
#print("Testing RF out, Press CTRL+C to exit")
class TakeoffDrone:
	def init(self):
		self.drone = None
		self.yaw = 0.0


	def convert_to_NED(self, forward, right, down):
		"""
		convert local right/forward motion to global north/east points
		"""
		theta = self.yaw * (math.pi / 180)
		north = forward * math.cos(theta) - right * math.sin(theta)
		east = forward * math.sin(theta) - right * math.cos(theta)
		return north, east, down

	def _map(x, in_min, in_max, out_min, out_max):
		return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

	async def run(self, system_addr):
    # Init the drone
		self.drone = System()
		await self.drone.connect(system_address=system_addr)
		GPIO.output(4,GPIO.HIGH)
		a = 1
    # Start the tasks

		print("Waiting for drone to have a global position estimate...")
		async for health in self.drone.telemetry.health():
			if health.is_global_position_ok:
				print("Global position estimate ok")
				display.lcd_display_string("Global pos estimate ok",2)
				display.lcd_display_string("                    ",2)
#            break
			if health.is_gyrometer_calibration_ok :
				print("gyrometer calibration ok")
				display.lcd_display_string("gyrometer cal ok",2)
				display.lcd_display_string("                    ",2)
#            break
			if health.is_accelerometer_calibration_ok:
				print("accelerometer calibration ok")
				display.lcd_display_string("accelerometer cal ok",2)
				display.lcd_display_string("                    ",2)
#            break
			if health.is_magnetometer_calibration_ok:
				print("magnetometer calibration ok")
				display.lcd_display_string("magnetometer cal ok",2)
				display.lcd_display_string("                    ",2)
#            break
			if health.is_local_position_ok:
				print("local position estimate ok")
				display.lcd_display_string("local position ok",2)
				display.lcd_display_string("                    ",2)
#            break
			if health.is_home_position_ok:
				print("home position estimate ok")
#				a=1
				display.lcd_display_string("home position ok",2)
				display.lcd_display_string("                    ",2)

#            break
			if health.is_armable:
				print("drone is armable")
				break

			break
#async def print_gps_info(drone):
		async for gps_info in self.drone.telemetry.gps_info():
			print(f"GPS info: {gps_info}")
			number = gps_info.num_satellites
			print(number)
			break


		async for position in self.drone.telemetry.position():
			print(position)
			print(f"Relative_alt: {position.relative_altitude_m}")
			display.lcd_display_string("Relative_Alt: %f"%position.relative_altitude_m, 3)
			display.lcd_display_string("                    ",3)
			break
		return number,a
		
		async def run1(self, system_addr):


			print("Code Initialized---")
		#self.drone = System()
		#await self.drone.connect(system_address="serial:///dev/ttyUSB0")
			status_text_task = asyncio.ensure_future(print_status_text(self.drone))
			asyncio.ensure_future(print_position(self.drone))

			print("Waiting for drone to connect...")
			async for state in self.drone.core.connection_state():
				if state.is_connected:
				print("Drone discovered!")
				break


			print("-- Arming")
			try:
				await self.drone.action.arm()
				await asyncio.sleep(3)
			except Exception as e:
				print(e)

			print("-- Fetching orientation")
			async for gps_info in self.drone.telemetry.heading():
				self.yaw = gps_info.heading_deg
				print(self.yaw)
				break
			print("-- Setting initial setpoint")
			await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

			print("-- Starting offboard")
			try:
				await self.drone.offboard.start()
			except OffboardError as error:
				print("-- Disarming")
				await self.drone.action.disarm()
				return

			print("-- takeoff to 1m")
			await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -1.5, self.yaw))
			await asyncio.sleep(5)


			while a == 1:
				values[0] = mcp.read_adc(0)
				y = _map(values[0],1,1023, 0 ,50)
				print("vale=%s"%y)
				n, e, d = self.convert_to_NED(0.0, 0.0, -y)
				await self.drone.offboard.set_position_ned(PositionNedYaw(n, e, d, self.yaw))
				await asyncio.sleep(10)	
				if GPIO.input(22)==0:
					print("a=%s"%a)
					print("-- Stopping offboard")
					await self.drone.offboard.stop()
					await asyncio.sleep(5)

					print("--Landing")
					await self.drone.action.land()
					await asyncio.sleep(10)
					a=0
			return a 
#    await drone.action.land()
#    await drone.action.takeoff()
			#return absolute_altitude,latitude,longitude
   



	async def print_status_text(self.drone):
		try:
		async for status_text in self.drone.telemetry.status_text():
			print(f"Status: {status_text.type}: {status_text.text}")
			display.lcd_display_string("%s:%s"%status_text.type %status_text.text,2)
			display.lcd_display_string("                    ",2)
		except asyncio.CancelledError:
			return

	async def print_position(self.drone):
		async for position in self.drone.telemetry.position():
			print(position)
			print(f"Relative_alt: {position.relative_altitude_m}")
			display.lcd_display_string("Relative_Alt: %f"%position.relative_altitude_m, 3)
			display.lcd_display_string("                    ",3)


	def _map(x, in_min, in_max, out_min, out_max):
		return int((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

while True:
		GPIO.output(4,GPIO.HIGH) #indication led
		values = [0]*8
		values[0] = mcp.read_adc(0)
		y = _map(values[0],1,1023, 0 ,50)
		print("value=%s"%y)
		display.lcd_display_string("Set Alt: %d"%y, 1)
		time.sleep(1)
		if GPIO.input(27) == 0 :
			loop = asyncio.get_event_loop()
			num,a1=loop.run_until_complete(drone.run(system_addr))
			while num > 10 and a1 == 1:
				b =  loop.run_until_complete(drone.run1(system_addr))
				a1=b
