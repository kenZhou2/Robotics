#!/user/bin/python
from PIL import Image 
import numpy as np
import math
import sys

def stackIm():
	

	print("very long runtime. most likely will run for 25 to 35 min")
	for num in range(1,7):
		f = "image"+str(num)+".jpg" 
		im = Image.open(f)
		width,height = im.size
		height = math.floor(height)
		width = math.floor(width)
		i = 1
		while i < 4:
			if i == 1: 
				im1 = np.array(im.crop((25,25,width - 15, height/3)))
				im12 = Image.fromarray(im1)
				#im12.show()
				#print(im1.shape)
			if i == 2: 
				im2 = np.array(im.crop((25,math.floor(height/3+26),width-15,(height/3)*2)))
				#print(im2.shape)
				#im12 = Image.fromarray(im2)
				#im12.show()
			if i == 3: 
				im3 = np.array(im.crop((25,math.floor(height/3)*2+16,width-15,height-10)))
				#print(im3.shape)
				#im12 = Image.fromarray(im3)
				#im12.show()
			i = i+1
		width,height = im12.size
		
	#im2 = np.array(im.crop((20,math.floor(height/3),width-12,(height/3)*2)))
		#print(type(height))
		#print(type(width))
		
		blurryImage = np.zeros((int(height),int(width),3),'uint8')


		for x in range(0,int(width)): 
			for y in range(0,int(height)): 
				blurryImage[y][x][0] = im3[y][x]
				blurryImage[y][x][1] = im2[y][x]
				blurryImage[y][x][2] = im1[y][x]
		
		unalignedIm = "image"+str(num)+"-color.jpg"


		image1_colored = Image.fromarray(blurryImage)
		#print(image1_colored.size)
		i,j = image1_colored.size
		#image1_colored.show()

		#image1_colored.save(unalignedIm )
		ssd(blurryImage,i,j,f,num)
		#image1_colored.show()
		#image1_colored.save(image)
	

def ssd(im, width, height, f,num):
	sumB = 0
	sumG = 0
	sumNccB = 0
	sumNccG = 0 
	w = int(width)
	h = int(height)
	area = int(w*h/2)
	shiftBx = 0
	shiftBy = 0
	shiftGx = 0
	shiftGy = 0
	shiftNBx = 0
	shiftNBy = 0
	shiftNGx = 0
	shiftNGy = 0

	im = im.astype(np.float64)

	nr = im[:,:,0]
	nb = im[:,:,1]
	ng = im[:,:,2]

	
	nim = np.zeros((int(height),int(width),3),'uint8')
	nim = nim.astype(np.float64)
	

	nr = (nr - np.average(nr)/(np.sqrt(np.sum([np.square(x - np.average(nr)) for x in nr]))))
	nb = (nb - np.average(nb)/(np.sqrt(np.sum([np.square(x - np.average(nb)) for x in nb]))))
	ng = (ng - np.average(ng)/(np.sqrt(np.sum([np.square(x - np.average(ng)) for x in ng]))))

	for x in range(0,int(width)): 
			for y in range(0,int(height)): 
				nim[y][x][0] = nr[y][x]
				nim[y][x][1] = nb[y][x]
				nim[y][x][2] = ng[y][x]
	#get first ssd value
	for x in range(0,w):
		for y in range(0,h):
			sumB = sumB + ((im[y][x][0]) - (im[y][x][1]))**2
			sumG = sumG + ((im[y][x][0]) - (im[y][x][2]))**2
			
			sumNccB = sumNccB + ((nim[y][x][0]) * (nim[y][x][1]))
			sumNccG = sumNccG + ((nim[y][x][0]) * (nim[y][x][2]))
	
	
	Bx = 0
	Gx = 0 
	rx1 = 7
	ry1 = 7

	rx2 = 7
	ry2 = 7
	bgx2 = 7
	bgx2 = 7
	
	rx3 = 7
	ry3 = 7
	bgx3 = 7
	bgy3 = 7
	
	imux = 7 
	imuy = 7 
	imdx = 7 
	imdy = 7 

	bgx = 7
	bgy = 7 


	for a in range(0,63):
		compareB = 0
		compareG = 0
		compareNB = 0
		compareNG = 0
		rx = 0 
		ry = 0 

	
		
		#-x and -y 
		for y in range(imuy,h):	
			for x in range(imux,w):
				compareB = compareB + ((im[ry][rx][0]) - (im[y][x][1]))**2
				compareG = compareG + ((im[ry][rx][0]) - (im[y][x][2]))**2
				compareNB = compareNB + ((nim[y][x][0]) * (nim[y][x][1]))
				compareNG = compareNG + ((nim[y][x][0]) * (nim[y][x][2]))
				#print("in x",x, "in y", y, "in rx",rx, "in ry", ry)
				rx += 1
			rx = 0
			ry += 1
		
		if compareB < sumB:
			sumB = compareB 
			shiftBx = imux * -1
			shiftBy = imuy * -1
		if compareG < sumG:
			sumG = compareG 
			shiftGx = imux * -1 
			shiftGy = imuy * -1

		if compareNB < sumNccB:
			sumNccB = compareNB 
			shiftNBx = imux * -1
			shiftNBy = imuy * -1
		if compareNG < sumNccG:
			sumNccG = compareNG 
			shiftNGx = imux * -1 
			shiftNGy = imuy * -1
		
		
		if imux > 0:		
			imux = imux - 1
		else: 
			imux = 7 
			imuy = imuy - 1 
		
		movex = 0 
		movey = 0 
		compareB = 0
		compareG = 0
		
		#-x and +y
		for j in range(ry2,h):	
			for i in range(bgx2,w):
				compareB = compareB + ((im[j][movex][0]) - (im[movey][i][1]))**2
				compareG = compareG + ((im[j][movex][0]) - (im[movey][i][2]))**2
				compareNB = compareNB + ((nim[j][movex][0]) * (nim[movey][i][1]))
				compareNG = compareNG + ((nim[j][movex][0]) * (nim[movey][i][2]))
				#print("in x",movex, "in y", movey)
				movex += 1
			movex = 0
			movey += 1
			

		if compareB < sumB:
			sumB = compareB 
			shiftBx = bgx2 * - 1
			shiftBy = ry2
			
		if compareG < sumG:
			sumG = compareG 
			shiftGx = bgx2 * - 1 
			shiftGy = ry2

		if compareNB < sumNccB:
			sumNccB = compareNB 
			shiftNBx = bgx2 * - 1
			shiftNBy = ry2
		if compareNG < sumNccG:
			sumNccG = compareNG 
			shiftNGx = bgx2 * - 1
			shiftNGy = ry2
		
		if bgx2 > 0:		
			bgx2 -= 1 
		else: 
			bgx2 = 7
			ry2 -= 1
		

		movex = 0 
		movey = 0 
		compareB = 0
		compareG = 0
		
		#+x and -y
		for j in range(bgy3,h):	
			for i in range(rx3,w):
				compareB = compareB + ((im[movey][i][0]) - (im[j][movex][1]))**2
				compareG = compareG + ((im[movey][i][0]) - (im[j][movex][2]))**2
				compareNB = compareNB + ((nim[movey][i][0]) * (nim[j][movex][1]))
				compareNG = compareNG + ((nim[movey][i][0]) * (nim[j][movex][2]))
				
				movex += 1
			movex = 0
			movey += 1
			
		if compareB < sumB:
			sumB = compareB 
			shiftBx = rx3
			shiftBy = bgy3*-1
			
		if compareG < sumG:
			sumG = compareG 
			shiftGx = rx3 
			shiftGy = bgy3*-1

		if compareNB < sumNccB:
			sumNccB = compareNB 
			shiftNBx = rx3
			shiftNBy = bgy3*-1
		if compareNG < sumNccG:
			sumNccG = compareNG 
			shiftNGx = rx3 
			shiftNGy = bgy3*-1

		if rx3 > 0:		
			rx3 -= 1 
		else: 
			rx3 = 7
			bgy3 -= 1

		movex1 = 0 
		movey1 = 0 
		compareB = 0
		compareG = 0

		#+x and +y
		
		for j in range(ry1,h):	
			for i in range(rx1,w):
				compareB = compareB + ((im[j][i][0]) - (im[movey1][movex1][1]))**2
				compareG = compareG + ((im[j][i][0]) - (im[movey1][movex1][2]))**2
				compareNB = compareNB + ((nim[j][i][0]) * (nim[movey1][movex1][1]))
				compareNG = compareNG + ((nim[j][i][0]) * (nim[movey1][movex1][2]))
				#print("in x",movex1, "in y", movey1, i,j)
				movex1 += 1
			movex1 = 0
			movey1 += 1 
		

		if compareB < sumB:
			sumB = compareB 
			shiftBx = rx1
			shiftBy = ry1
			
		if compareG < sumG:
			sumG = compareG 
			shiftGx = rx1
			shiftGy = ry1

		if compareNB < sumNccB:
			sumNccB = compareNB 
			shiftNBx = rx1
			shiftNBy = ry1
		if compareNG < sumNccG:
			sumNccG = compareNG 
			shiftNGx = rx1 
			shiftNGy = ry1
		
		
		if rx1 > 0:		
			rx1 -= 1
		else: 
			rx1 = 7 
			ry1 -= 1 
		

		#print(a)

	cImage = np.zeros((h,w,3),'uint8')
	for x in range(0,w): 
		for y in range(0,h): 
			cImage[y][x][0] = im[y][x][0] 
	
	#works 
	if shiftGx <= 0 and shiftGy <= 0:
		for y in range(0, h - shiftGy*-1):
			for x in range (0, w - shiftGx*-1):
				cImage[y][x][2] = im[y+ shiftGy*-1][x+ shiftGx*-1][2]
			
	
	#works
	elif shiftGx >= 0 and shiftGy <= 0:
		for y in range(shiftGy*-1, h):
			for x in range (shiftGx,w):
				cImage[y - shiftGy*-1][x][2] = im[y][x - shiftGx][2]
			
	#works
	elif shiftGx >= 0 and shiftGy >= 0:	
		for y in range(shiftGy, h):
			for x in range (shiftGx, w):
				cImage[y][x][2] = im[y - shiftGy][x - shiftGx][2]
				

	#works
	elif shiftGx <= 0 and shiftGy >= 0:
		for y in range(shiftGy, h):
			for x in range (shiftGx*-1,w):
				cImage[y][x-shiftGx*-1][2] = im[y-shiftGy][x][2]

	

	#works
	if shiftBx <= 0 and shiftBy <= 0:
		for y in range(0, h - shiftBy*-1):
			for x in range (0, w - shiftBx*-1):
				cImage[y][x][1] = im[y + shiftBy*-1][x + shiftBx*-1][1]
				
	
	#works
	elif shiftBx >= 0 and shiftBy <= 0:
		for y in range(shiftBy*-1, h):
			for x in range (shiftBx, w):
				cImage[y - shiftBy*-1][x][1] = im[y][x- shiftBx][1]
				
	#works
	elif shiftBx >= 0 and shiftBy >= 0:
		for y in range(shiftBy, h):
			for x in range (shiftBx, w):
				cImage[y][x][1] = im[y-shiftBy][x-shiftBx][1]
			

	#works
	elif shiftBx <= 0 and shiftBy >= 0:
		for y in range(shiftBy,h):
			for x in range (shiftBx*-1,w):
				cImage[y][x-shiftBx*-1][1] = im[y-shiftBy][x][1]
				
	image_colored = Image.fromarray(cImage)
	
	image_colored.show()

	print(f)
	print("SSD Alignment")
	print("Green:", "[",str(shiftGy),str(shiftGx),"]")
	print("Blue:","[",str(shiftBy),str(shiftBx),"]")
	f = "image" + str(num) + "-ssd.jpg"
	image_colored.save(f)

	

	
	nccImage = np.zeros((h,w,3),'uint8')
	for x in range(0,w): 
		for y in range(0,h): 
			nccImage[y][x][0] = im[y][x][0] 
	
	#works
	if shiftNBx <= 0 and shiftNBy <= 0:
		for y in range(0, h - shiftNBy*-1):
			for x in range (0, w - shiftNBx*-1):
				nccImage[y][x][1] = im[y + shiftNBy*-1][x + shiftNBx*-1][1]
				
	
	#works
	elif shiftNBx >= 0 and shiftNBy <= 0:
		for y in range(shiftNBy*-1, h):
			for x in range (shiftNBx, w):
				nccImage[y - shiftNBy*-1][x][1] = im[y][x- shiftNBx][1]
				
	#works
	elif shiftNBx >= 0 and shiftNBy >= 0:
		for y in range(shiftNBy, h):
			for x in range (shiftNBx, w):
				nccImage[y][x][1] = im[y-shiftNBy][x-shiftNBx][1]
			

	#works
	elif shiftNBx <= 0 and shiftNBy >= 0:
		for y in range(shiftNBy,h):
			for x in range (shiftNBx*-1,w):
				nccImage[y][x-shiftNBx*-1][1] = im[y-shiftNBy][x][1]




	#works 
	if shiftNGx <= 0 and shiftNGy <= 0:
		for y in range(0, h - shiftNGy*-1):
			for x in range (0, w - shiftNGx*-1):
				nccImage[y][x][2] = im[y+ shiftNGy*-1][x+ shiftNGx*-1][2]
			
	
	#works
	elif shiftNGx >= 0 and shiftNGy <= 0:
		for y in range(shiftNGy*-1, h):
			for x in range (shiftNGx,w):
				nccImage[y - shiftNGy*-1][x][2] = im[y][x - shiftNGx][2]
			
	#works
	elif shiftNGx >= 0 and shiftNGy >= 0:	
		for y in range(shiftNGy, h):
			for x in range (shiftNGx, w):
				nccImage[y][x][2] = im[y - shiftNGy][x - shiftNGx][2]
				

	#works
	elif shiftNGx <= 0 and shiftNGy >= 0:
		for y in range(shiftNGy, h):
			for x in range (shiftNGx*-1,w):
				nccImage[y][x-shiftNGx*-1][2] = im[y-shiftNGy][x][2]
	
				
	image_colored = Image.fromarray(nccImage)
	
	image_colored.show()

	print("NCC Alignment")
	print("Green: [", str(shiftNGy),str(shiftNGx),"]")
	print("Blue:","[",str(shiftNBy),str(shiftNBx),"]")
	
	f = "image" + str(num) + "-ncc.jpg"
	image_colored.save(f)

	
	print("")
		
			

if __name__ == '__main__': 	
	stackIm()
	
	

