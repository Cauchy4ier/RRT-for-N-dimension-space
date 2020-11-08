from abc import ABCMeta, abstractmethod

import numpy as np


class CollisionObject(object):
    """
    Abstract class for a parametrically defined collision object.
    """
    __metaclass__= ABCMeta
    @abstractmethod
    def __init__(self):
        pass
    def in_collision(self, target):
        """
        Checks whether target point is in collision. Points at the boundary of
        the object are in collision.

        :returns: Boolean indicating target is in collision.
        """
        pass


class CollisionBox(CollisionObject):
    """
    N-dimensional box collision object.
    """
    def __init__(self, location, half_lengths):
        """
        :params location: coordinates of the center
        :params half_lengths: half-lengths of the rectangle along each axis
        """
        super(CollisionBox, self).__init__()
        self.location = np.asarray(location)
        self.half_lengths = np.asarray(half_lengths)
        self.ndim = self.location.shape[0]

    def in_collision(self, target):
        super(CollisionBox, self).in_collision(target)
        dis=0
        for i in range(0,self.location.size):    # collide if for all dismensions,distance between target and center is bigger than half_length on that dimension
         dis=np.abs(self.location[i]-target[i])-self.half_lengths[i]
         if(dis>0):return False

        return True

class CollisionSphere(CollisionObject):
    """
    N-dimensional sphere collision object.
    """
    def __init__(self, location, radius):
        """
        :params location: coordinates of the center
        :params radius: radius of the circle
        """
        self.location = np.asarray(location)
        self.radius = radius

    def in_collision(self, target):
        dis=0
        for i in range(0,self.location.size):       # collide if the distance between target and center is smaller than radius
         dis+=(self.location[i]-target[i])**2

        dis=np.sqrt(dis)
        return (dis<=self.radius)
