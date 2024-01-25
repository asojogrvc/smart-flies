def set_bit(v: int, index:int, x:int) -> int:
  """
  Set the index:th bit of v to 1 if x is truthy, else to 0, and return the new value.
  """
  mask = 1 << index-1   # Compute mask, an integer with just bit 'index' set.
  v &= ~mask          # Clear the bit indicated by the mask (if x is False)
  if x:
    v |= mask         # If x was True, set the bit indicated by the mask.

  return v            # Return the result, we're done

def is_set(v:int, n:int) -> bool:
    return v & 1 << n != 0

num = 0b1000

num = set_bit(num, 1, 1)
print(bin(num))
num = set_bit(num, 2, 1)
print(bin(num))