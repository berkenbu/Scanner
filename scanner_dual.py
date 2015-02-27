#!/opt/local/bin/python2.7

#!/usr/bin/python

import sys
import cv2
import serial
import time
import math
import copy

class Steppers:
    def __init__(self, port, baud=38400):
        self.serial = serial.Serial(port, baud)
        self.Nsteps = 2048
        self.laser = 0

    def step (self, x, y):
        """ Take one step in x and y direction """
        return self._serial_command("X{:.1f}Y{:.1f}\n".format(x, y))

    def laser_off(self):
        """ Turn laser off """
        if (self.laser == 0): return True
        if (self._serial_command("S 0\n")):
            self.laser = 0
            return True
        else: 
            return False
    
    def laser_on(self, level):
        """ Turn laser on """
        if (self.laser == level): return True
        if (self._serial_command("S "+str(level)+"\n")):
            self.laser = level
            return True
        else:
            return False

    def home(self):
        return self._serial_command("H\n")

    def _serial_command (self, cmd):
        self.serial.write(cmd)
        self.serial.flush()
        answer = self.serial.readline()
        return (answer.startswith("OK"))
        

class Matrix:
    def __init__ (self, m):
        if (m == None):
            self.m = [[1.0 if (i==j) else 0.0 for j in range(3)] for i in range(3)]
        else:
            self.m = copy.deepcopy(m)
        
    def __str__ (self):
        return "["+str(self.m[0][0])+","+str(self.m[0][1])+","+str(self.m[0][2])+"]\n["+str(self.m[1][0])+","+str(self.m[1][1])+","+str(self.m[1][2])+"]\n["+str(self.m[2][0])+","+str(self.m[2][1])+","+str(self.m[2][2])+"]\n"

    def fromVec (self, v1, v2, v3):
        m = [[0 for i in range(3)] for j in range(3)]
        m[0] = [v1.x, v2.x, v3.x]
        m[1] = [v1.y, v2.y, v3.y]
        m[2] = [v1.z, v2.z, v3.z]
        return Matrix(m)
    
    def vmult (self, v):
        return Vec(self.m[0][0]*v.x+self.m[0][1]*v.y+self.m[0][2]*v.z,self.m[1][0]*v.x+self.m[1][1]*v.y+self.m[1][2]*v.z,self.m[2][0]*v.x+self.m[2][1]*v.y+self.m[2][2]*v.z)

    def mmult (self, m):
        tmp = [[0 for i in range(3)] for j in range(3)]
        for i in range(3):
            for j in range(3):
                for k in range(3):
                    tmp[i][j] += self.m[i][k]*m.m[k][j]
        return Matrix(tmp)
    
    def transpose (self):
        tmp = [[self.m[j][i] for j in range(3)] for i in range(3)]
        return Matrix(tmp)

    def zRotation (self, alpha):
        print >>sys.stderr, "alpha="+str(alpha)
        m = [[1.0 if (i==j) else 0.0 for j in range(3)] for i in range(3)]
        m[0][0] = math.cos(alpha)
        m[0][1] = -math.sin(alpha)
        m[1][0] = math.sin(alpha)
        m[1][1] = math.cos(alpha)
        return Matrix(m)

    def yRotation (self, alpha):
        m = [[1.0 if (i==j) else 0.0 for j in range(3)] for i in range(3)]
        m[0][0] = math.cos(alpha)
        m[0][2] = -math.sin(alpha)
        m[2][0] = math.sin(alpha)
        m[2][2] = math.cos(alpha)
        return Matrix(m)


class Vec:
    def __init__ (self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __str__ (self):
        return "("+str(self.x)+","+str(self.y)+","+str(self.z)+")"

    def add (self, v):
        return Vec(self.x+v.x, self.y+v.y, self.z+v.z)
    
    def sub (self, v):
        return Vec(self.x-v.x, self.y-v.y, self.z-v.z)

    def norm (self):
        return math.sqrt(self.x*self.x+self.y*self.y+self.z*self.z)

    def normalize (self):
        if (self.norm()>0):
            return Vec(self.x/self.norm(), self.y/self.norm(), self.z/self.norm())
        else:
            return self

    def inner (self, v):
        return self.x*v.x + self.y*v.y + self.z*v.z
    
    def smult (self, a):
        return Vec(a*self.x, a*self.y, a*self.z)

    def cross (self, v):
        return Vec(self.y*v.z-self.z*v.y, self.z*v.x-self.x*v.z, self.x*v.y-self.y*v.x)

    def distance (self, v):
        d = self.sub(v)
        return math.sqrt(d.x*d.x+d.y*d.y+d.z*d.z)


class CameraProperties:
    def __init__(self, res_x=480, res_y=640, fx=0.6, fy=0.6):
        #fx=11*25.4/380
        self.res_x = res_x
        self.res.y = res_y
        self.fx = fx
        self.fy = fy

class Geometry:
    """
  Reference frame for everything is 
   Origin: center of platter
   X-axis: (foot) of laser to Origin
   Y-axis: right-handed coordinate system
   Z-axis: orthogonal to platter
   all units in mm

"""
    def __init__(self, cp=None, cam=None):
        if (cp!=None):
            self.cam_pos = cp
        else:
            self.cam_pos = Vec(-130.0, -170.0, 121.0)
        if (cam!=None):
            self.cam = cam
        else:
            self.cam = Camera()
        self.laser_pos = Vec(-283.0, 0.0, 103.0)
        self.platter_r = 60.0
        cd = Vec(self.cam_pos.x, self.cam_pos.y, self.cam_pos.z).smult(-1.0).normalize()
        cd_o = Vec(0, 0, 1)
        cdx = cd.cross(cd_o).normalize()
        cd_o = cd.cross(cdx)
        self.camMat = Matrix(None).fromVec(cdx, cd, cd_o)
        
    def toCameraFrame(self, v):
        return self.camMat.transpose().vmult(v)

    def fromCameraFrame(self, v):
        return self.camMat.vmult(v)
        
    def cameraHomeDir(self):
        return Vec(-self.platter_r-self.cam_pos.x, -self.cam_pos.y, -self.cam_pos.z).normalize()
        
    def laserHomeDir(self):
        return Vec(-self.platter_r-self.laser_pos.x, 0, -self.laser_pos.z).normalize()

    def camDirFromPixelPos(self, x, y):
        return Vec(self.cam.fx*(self.cam.res_x/2-x)/self.cam.res_x, 1.0, self.cam.fy*(y-self.cam.res_y/2)/self.cam.res_y).normalize()

    def pixelPosFromCamDir(self, v):
        v = v.smult(1.0/v.y)
        x = self.cam.res_x/2 - self.cam.res_x*v.x/self.cam.fx
        y = self.cam.res_y/2 + self.cam.res_y*v.z/self.cam.fy
        return x, y


class DotFinder:
    def __init__(self, cam_num, cam_pos=None, cam_props=None):
        self.capture = cv2.VideoCapture(cam_num)
        self.geom = Geometry(cam_pos, cam_props)

    def getImage(self):
        _,img = self.capture.read()
        return img

    def findDot(self, image):
        #cv2.setImageCOI(image, 1) # red channel
        (blue,green,red) = cv2.split(image)
        coi = 0.2*blue + 0.2*green + 0.6*red
        (minVal,maxVal,minLoc,maxLoc) = cv2.minMaxLoc(coi)
        (x,y) = maxLoc
        return (x,y)

    def findDot2(self, image):
        if (image==None):
            image = self.getImage()
            image = cv2.flip(cv2.transpose(image),1)
            image = cv2.blur(image, (2, 2))
        (blue,green,red) = cv2.split(image)
        coi = 0.15*blue + 0.15*green + 0.7*red
        coi = cv2.convertScaleAbs(coi)
        ret, thresh = cv2.threshold(coi, 210, 255, cv2.THRESH_BINARY)
        dist = cv2.distanceTransform(thresh, cv2.cv.CV_DIST_L1, 3)
        dist = cv2.convertScaleAbs(dist)
        (minVal,maxVal,minLoc,maxLoc) = cv2.minMaxLoc(dist)
        (x,y) = maxLoc
        return (x,y,image)


class Scanner:
    def __init__(self, finder1, finder2, stepper):
        self.finder1 = finder1
        self.finder2 = finder2
        self.stepper = stepper

    def rayXYPlaneIntersection(self, p, u):
        tc = -p.y/u.y
        intersec = p.add(u.smult(tc))
        return intersec, tc

    def rayRayIntersection(self, p, u, q, v):
        # aux definitions
        w0 = p.sub(q)
        a = u.inner(u)
        b = u.inner(v)
        c = v.inner(v)
        d = u.inner(w0)
        e = v.inner(w0)
        # closest approach ray parameters sc and tc
        sc = (b*e - c*d)/(a*c - b*b)
        tc = (a*e - b*d)/(a*c - b*b)
        # resulting closest approach points, their average and their distance
        pc = p.add(u.smult(sc))
        qc = q.add(v.smult(tc))
        ave_intersec = pc.add(qc).smult(0.5)   # average of the two closest points on laser and camera ray
        dist = pc.distance(qc)
        return ave_intersec,pc,qc,dist

    def scan(self):
        alpha = 0  # angle of rotation
        z_steps = 256
        y_steps = 70
        last_point = None
        self.stepper.home()
        for zstep in range(0,z_steps):
            platter_rot_matrix = Matrix(None).zRotation(zstep*2*math.pi/z_steps)
            for ystep in range(0,y_steps):
                (dot_x, dot_y,img) = self.finder1.findDot2()
                cv2.circle(img, (dot_x, dot_y), 3, (0, 255, 0))
                u1 = self.finder1.geom.fromCameraFrame(self.finder1.geom.camDirFromPixelPos(dot_x, dot_y))
                (point1, _) = self.rayXYPlaneIntersection(self.finder1.geom.cam_pos, u1)

                (dot_x, dot_y,img) = self.finder2.findDot2()
                cv2.circle(img, (dot_x, dot_y), 3, (0, 0, 255))
                u2 = self.finder2.geom.fromCameraFrame(self.finder2.geom.camDirFromPixelPos(dot_x, dot_y))
                (point2, _) = self.rayXYPlaneIntersection(self.finder2.geom.cam_pos, u2)

                

                point = point1
                point = platter_rot_matrix.vmult(point)
                print str(point.x) + " " + str(point.y) + " " + str(point.z)
                self.stepper.step(0,-1)
                cv2.imshow("Scanner", img)
                cv2.waitKey(20)
            self.stepper.step(self.stepper.Nsteps/z_steps, y_steps)
        return
        

def main():
    stepper = Steppers('/dev/cu.usbmodem212021')
    stepper.laser_on(90)
    finder1 = DotFinder(0, Vec(-130.0, -170.0, 121.0), CameraProperties())
    finder2 = DotFinder(1, Vec(-130.0, 170.0, 121), CameraProperties())
    scanner = Scanner(finder1, finder2, stepper)  
    cv2.namedWindow("Scanner", cv2.CV_WINDOW_AUTOSIZE)
    scanner.scan()
   

if __name__ == '__main__':
        main()
