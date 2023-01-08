import math



for i in range(91):
  if i%10 == 0:
    print()
  range = math.tan((i/1)*2*math.pi/360)
  print( str(round(range,4)),end=", ")
  