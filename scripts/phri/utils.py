

import numpy as np
import cv2
import matplotlib.pyplot as plt
import tf.transformations as tfs
import warnings

def showMarker(im):
    plt.imshow(im, interpolation='nearest', cmap='gray')

def read_image(fname):
    im = cv2.imread(fname)
    im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)
    return im




"""
class tgenerate(object):
	def __init__():
	def g():
"""
eps = 1e-4

# get skew symmetric matrix of w or twist
def _hat(vec):

	if vec.size==3:
		w = vec
		vec_hat = np.array([
			[0.0, -w[2], w[1]],
			[w[2], 0.0, -w[0]],
			[-w[1], w[0], 0.0]])
	elif vec.size==6:
		v = vec[:3]
		w = vec[3:]
		vec_hat = np.array([
			[0.0, -w[2], w[1], v[0]],
			[w[2], 0.0, -w[0], v[1]],
			[-w[1], w[0], 0.0, v[2]],
			[0.0, 0.0, 0.0, 0.0]])
	else:
		vec_hat = None
		warnings.warn("Input must be of size 3 or 6.")

	return vec_hat

# get vector from 3x3 or 4x4 skew symmetric matrix
def _vee(mat):
	
	if mat.size==9:
		vec = np.array([mat[2,1], mat[0,2], mat[1,0]])
	
	elif mat.size==16:
		vec = np.array(
			[mat[0,3], mat[1,3], mat[2,3], 
			mat[2,1], mat[0,2], mat[1,0]])
	else:
		vec = None	
		warnings.warn("Input must be of size 9 or 16.")
	return vec

# Return homogeneous rotation matrix from quaternion
def rot_from_quat(quat):
	return tfs.quaternion_matrix(quat)

def g_from_qv(quat,vec):
	rot = rot_from_quat(quat)
	trmat = translation_matrix(vec)
	g = np.matmul(trmat,rot)
	return g

def quat_from_R(R):
    '''
    @param R: 3x3 np array
	'''
    R = np.append(np.append(R, [[0,0,0]], axis=0), 
			[[0], [0], [0], [1]], axis=1)
    quat = tfs.quaternion_from_matrix(R)
    return quat

# def quat_from_g(g):
# 	return quat

def translation_matrix(vec):
	return tfs.translation_matrix(vec)

def inverse(mat):
	return tfs.inverse_matrix(mat)

# from given pose (example: cartesian coordinates of end effector)
# finds homogeneous transformation matrix
def g_from_pose(pose):

	if type(pose)==dict:
		vec = pose['position']
		quat = pose['orientation']
	elif type(pose)==np.ndarray:
		vec = pose[:3]
		quat = pose[3:]

	g = np.matmul(translation_matrix(np.asarray(vec)), 
		rot_from_quat(np.asarray(quat)))
	g[abs(g)<eps] = 0.0
	return g


def pose_from_g(g):
	pos = g[:3.3].flatten()
	quat = tfs.quaternion_from_matrix(g)

	pose = {
		'position':pos,
		'orientation': quat
	}

	return pose

def expcoord(mat):
	
	
	if mat.size == 9:
		mat = np.append(np.append(mat, [[0,0,0]], axis=0), 
			[[0], [0], [0], [1]], axis=1)

	R = mat[0:3,0:3];
	angle, direc, point = tfs.rotation_from_matrix(mat)
	p = tfs.translation_from_matrix(mat)

	A = (np.matmul((np.eye(3)-R), _hat(direc))
		+ angle*np.matmul(np.reshape(direc,(3,1)), np.reshape(direc,(1,3))))

	v = np.linalg.solve(A,p)

	return np.concatenate((v, direc))*angle

def g_from_expcoord(expcoord):
	
	a = expcoord[:3]
	b = expcoord[3:]

	angle = tfs.vector_norm(b)
	direc = np.zeros(3)
	R = np.eye(4)
	if angle != 0.0:
		direc = tfs.unit_vector(b)
		R = tfs.rotation_matrix(angle,direc)

		v = a/angle
		A = (np.matmul((np.eye(3)-R[:3,:3]), _hat(direc)) + 
			angle*np.matmul(np.reshape(direc,(3,1)), np.reshape(direc,(1,3))))
		p = np.matmul(A,v)

		return np.matmul(tfs.translation_matrix(p),R)
	elif angle == 0.0:
		return tfs.translation_matrix(a)

def main():
	
	return

if __name__=='__main__':

	main()
'''
	sigma = utils.expcoord(np.matmul(g0[:3,:3].T,gf[:3,:3]))
	prod = np.matmul(np.matmul(g0[:3,:3], utils._hat(sigma)),g0[:3,:3].T)
		ft = a*t**3+b*t**2
	ft_dot = 3*a*t**2 + 2*b*t
#	w = (1-ft/Tf)*xi0[3:] + ft/Tf*xif[3:]
	w_dot = -ft_dot/Tf*prod
	p = (1-ft/Tf)*p0 + ft/Tf*pf
	p_dot = -ft_dot/Tf*(p0-pf)
	lin = -np.matmul(w_dot, p) + p_dot
'''