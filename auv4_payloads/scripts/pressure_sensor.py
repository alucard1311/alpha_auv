import ms5837

sensor = ms5837.MS5837_30BA(0) # Use default I2C bus (1)

if sensor.init():
	print(" initialized")

if not sensor.read(ms5837.OSR_512):
	print("No Data")

sensor.setFluidDensity(1000)



print(sensor.depth())
