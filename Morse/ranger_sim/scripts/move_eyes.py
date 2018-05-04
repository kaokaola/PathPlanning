from pymorse import Morse  

print("Use wasd to control the Ranger")

with Morse() as simu:

  motion = simu.robot.motion
  pose = simu.robot.pose
  eyes = sium.robot.eyes

  v = 0.0
  w = 0.0
  left = 0.0
  right = 0.0

  while True:
      key = input("WASD?")

      if key.lower() == "w":
          v += 0.5
      elif key.lower() == "s":
          v -= 0.5
      elif key.lower() == "a":
          w += 0.5
      elif key.lower() == "d":
          w -= 0.5

      if key.lower() == "r":
          left += 0.5
      elif key.lower() == "f":
          left -= 0.5
      elif key.lower() == "t":
          right += 0.5
      elif key.lower() == "g":
          right -= 0.5

      else:
          continue

      print("The robot is currently at: %s" % pose.get())

      motion.publish({"v": v, "w": w})
      motion.publish({"left": left, "right": right})
