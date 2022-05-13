
img = "Image.txt"

with open(img) as f:
    flist = f.read().splitlines()
    print flist
    fflist = flist[0].split()
    print fflist
    print ' '.join(fflist)
    
