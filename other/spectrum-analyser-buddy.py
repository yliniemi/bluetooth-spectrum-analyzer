import math

def drawSpectrum(startingPoint, bins, ratio, previous, printThem):
    if printThem:
        print(f"{bins} {startingPoint} - {previous}")
    if bins < 2:
        return startingPoint;
    return drawSpectrum(math.floor(startingPoint * ratio - 0.5), bins - 1, ratio, startingPoint - 1, printThem)

def binaryRatio(ratio, tooLow, tooHigh, depth, startingPoint, bins):
    if depth < 1:
        # print(f"StartingPoint")
        drawSpectrum(startingPoint, bins, tooHigh, startingPoint, True)
        print(f"Optimal ratio: {tooHigh}")
        return tooHigh
    if drawSpectrum(startingPoint, bins, ratio, startingPoint, False) < 1:
        return binaryRatio((ratio + tooHigh) / 2, ratio, tooHigh, depth - 1, startingPoint, bins)
    else:
        return binaryRatio((ratio + tooLow) / 2, tooLow, ratio, depth - 1, startingPoint, bins)


# drawSpectrum(1024, 32, 0.16)
# drawSpectrum(768, 32, 0.151, 1024)
# drawSpectrum(652, 32, 0.151, 1024) # best for old my ears
# print(drawSpectrum(768, 32, 0.844, 1023))
# print(drawSpectrum(384, 32, 0.87, 511))
# print(drawSpectrum(192, 32, 0.891, 255))

# print(binaryRatio(0.5, 0, 1, 30, 768, 32))
# print(binaryRatio(0.5, 0, 1, 30, 384, 32))
# print(binaryRatio(0.5, 0, 1, 30, 192, 32))
# print(binaryRatio(0.5, 0, 1, 30, 768, 48))
# print(binaryRatio(0.5, 0, 1, 30, 768, 64))
# print(binaryRatio(0.5, 0, 1, 30, 768, 16))

# 512 fft took 700 us
# 1024 fft took 905 us
# 2048 fft took 1369 us


# drawSpectrum(2048, 120, 0.94,2048, True)


binaryRatio(0.5, 0, 1, 30, 2048 * 0.8, 65)
print()
binaryRatio(0.5, 0, 1, 30, 2048 * 0.8, 49)
print()
binaryRatio(0.5, 0, 1, 30, 2048 * 0.8, 33)
print()
binaryRatio(0.5, 0, 1, 30, 2048 * 0.8, 17)
print()
binaryRatio(0.5, 0, 1, 30, 2048 * 0.8, 8)
print()
binaryRatio(0.5, 0, 1, 30, 2048 * 0.8, 113)
print()
binaryRatio(0.5, 0, 1, 30, 2048 * 0.8, 115)
print()
print()

binaryRatio(0.5, 0, 1, 30, 2048 * 0.8, 129)
print()
binaryRatio(0.5, 0, 1, 30, 2048 * 0.9, 129)
print()
binaryRatio(0.5, 0, 1, 30, 2048 * 1, 129)
print()
print()

