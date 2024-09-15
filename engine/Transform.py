import torch

import engine.Quaternion as Quaternion

FORWARD = torch.FloatTensor([[0, 1, 0]]).cuda()
RIGHT = torch.FloatTensor([[1, 0, 0]]).cuda()
UP = torch.FloatTensor([[0, 0, 1]]).cuda()
BACKWARD = torch.FloatTensor([[0, -1, 0]]).cuda()
LEFT = torch.FloatTensor([[-1, 0, 0]]).cuda()
DOWN = torch.FloatTensor([[0, 0, -1]]).cuda()
ZEROS = torch.zeros((1, 3)).cuda()
ONES = torch.ones((1, 3)).cuda()

def GetUnit(vector):
	magnitude = torch.linalg.norm(vector)

	if magnitude == 0:
		magnitude = 1

	return vector / magnitude

def RotationToDirection(rotation):
	x = -torch.sin(rotation[2]) * torch.cos(rotation[0])
	y = torch.cos(rotation[2]) * torch.cos(rotation[0])
	z = torch.sin(rotation[0])

	direction = torch.FloatTensor([x, y, z]).cuda()
	magnitude = torch.linalg.norm(direction)

	return direction / magnitude

def RotateDirection(quaternion, direction):
	direction_rotated = Quaternion.RotatePoints(direction, quaternion)

	magnitude = torch.linalg.norm(direction_rotated)
	if magnitude == 0:
		magnitude = 1

	return direction_rotated / magnitude

def GetForward(quaternion):
	return RotateDirection(quaternion, FORWARD)

def GetRight(quaternion):
	return RotateDirection(quaternion, RIGHT)

def GetUp(quaternion):
	return RotateDirection(quaternion, UP)

def GetBackward(quaternion):
	return RotateDirection(quaternion, BACKWARD)

def GetLeft(quaternion):
	return RotateDirection(quaternion, LEFT)

def GetDown(quaternion):
	return RotateDirection(quaternion, DOWN)
