
class Wheel:
    def __init__(self,wheel_id, wheel_name):
        self.id = wheel_id
        self.name = wheel_name
        self.rpm = 0
        self.direction = 0
        self.orientation = 0 # 0 is +x and counted in anticlockwise direction around z axis
        self.pos = [0.0]*7
        self.pos[-1] = 1.0

    def set_orientation(self,ornt):
        self.orientation = ornt

    def set_direction(self, direct):
        self.direction = direct 
    
    def set_rpm(self, rpm):
        self.rpm = rpm

