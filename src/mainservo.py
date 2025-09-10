from servo import make_servo_and_limiter

servo, limiter = make_servo_and_limiter()

# ... your perception / wall & pillar logic sets:
#   angle (deg), lTurn (bool), rTurn (bool)

angle_cmd = limiter.apply(angle, lTurn, rTurn)
servo.set_deg(angle_cmd)
servo.tick()  # call every loop (~200–300 Hz is fine)

# on round end or shutdown:
servo.center()
servo.close()
