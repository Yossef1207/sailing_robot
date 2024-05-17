import cv2   
# Read the image

for i in range(1,10):
	img = cv2.imread('video_shot_' + str(i) + '.png')
	print(img.shape)
	height = img.shape[0]
	width = img.shape[1]

	# Cut the image in half
	width_cutoff = width // 2
	img_right = img[:, :width_cutoff] # right
	img_left = img[:, width_cutoff:] # left

	cv2.imwrite("../right_cut/video_right_" + str(i) + ".png", img_right)
	cv2.imwrite("../left_cut/video_left_" + str(i) + ".png", img_left)
