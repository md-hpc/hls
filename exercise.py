import random
import hls
import yaml

N = 100

m = hls.MockFPGA()

mem = m.add(hls.BRAM(N,iports=0,oports=1))
maximum = m.add(hls.Register())
minimum = m.add(hls.Register())

'''
Put your solution here
'''

arr = [random.random() for _ in range(N)]

mem.contents = arr.copy()
maximum.contents = -1
minimum.contents = -1

while maximum.contents == -1 and minimum.contents == -1:
    m.clock()

if maximum.contents == max(arr) and minimum.contents == min(arr):
    print("Success!")
else:
    print(f"max: {max(arr)} (computed {maximum.contents})")
    print(f"min: {min(arr)} (computed {minimum.contents})")
