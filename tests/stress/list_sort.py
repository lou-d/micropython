# test large list sorting (should not stack overflow)
l = list(range(2000))
l.sort()
print(l[0], l[-1])
l.sort(reverse=True)
print(l[0], l[-1])