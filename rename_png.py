import os
import glob
datadir = "./data/230207_megane/pngtest/*.jpg"
flist = glob.glob(datadir)
i = 0
# ファイル名を一括で変更する
for file in sorted(flist):
    print(file)
    os.rename(file, './data/230207_megane/pngtest/' + str(i) + '.png')
    i+=1