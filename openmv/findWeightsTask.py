import sensor, image, time

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

black = (0, 20, -5, 5, -5, 5)

# 找最大色块函数
def FindMaxBlob (blobs):
    BlobsAreaMax = 0
    MaxBlob = None
    for blob in blobs:
        if blob[2]*blob[3] > BlobsAreaMax:
            MaxBlob = blob
            BlobsAreaMax = blob[2]*blob[3]
    return MaxBlob

# 判断中心是否在合理范围
def centreCheck(x, y):
    if(132<=x<=235 and 31<=y<=134):
        return True
    else:
        return False

while(True):

    img = sensor.snapshot()
    blackBlobs = img.find_blobs([black])
    maxBlob = FindMaxBlob(blackBlobs)
    if(maxBlob != None):
        centrecheck = centreCheck(maxBlob[5], maxBlob[6])
    if(maxBlob != None and centrecheck):
        img.draw_rectangle(maxBlob.rect())
