import open3d.core as o3c
import numpy as np

# Tensor from list.
a = o3c.Tensor([0, 1, 2])
print("Created from list:\n{}".format(a))

# Tensor from Numpy.
a = o3c.Tensor(np.array([0, 1, 2]))
print("\nCreated from numpy array:\n{}".format(a))

# Dtype and inferred from list.
a_float = o3c.Tensor([0.0, 1.0, 2.0])
print("\nDefault dtype and device:\n{}".format(a_float))

# Specify dtype.
a = o3c.Tensor(np.array([0, 1, 2]), dtype=o3c.Dtype.Float64)
print("\nSpecified data type:\n{}".format(a))

# Specify device.
# a = o3c.Tensor(np.array([0, 1, 2]), device=o3c.Device("CUDA:0"))
a = o3c.Tensor(np.array([0, 1, 2]), device=o3c.Device("CPU:0"))
print("\nSpecified device:\n{}".format(a))


vals = np.array([1, 2, 3])
src = o3c.Tensor(vals)
dst = src
print("\ndst:\n{}".format(dst))
src[0] += 10

print("src:\n{}".format(src))
print("\ndst:\n{}".format(dst))

print("\nPROPERTIES OF A TENSOR")
vals = np.array((range(24))).reshape(2, 3, 4)
a = o3c.Tensor(vals, dtype=o3c.Dtype.Float64, device=o3c.Device("CPU:0"))
print(f"a.shape: {a.shape}")
print(f"a.strides: {a.strides}")
print(f"a.dtype: {a.dtype}")
print(f"a.device: {a.device}")
print(f"a.ndim: {a.ndim}")
