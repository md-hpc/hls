from structures import *
from hls import NULL

def bram_enum(cache, double_buffer):
    a0 = 0 if not double_buffer else DBSIZE
    a1 = a0 + DBSIZE
    for addr, val in enumerate(cache.contents[a0:a1]):
        yield addr + a0, val

def verify(inputs, p_caches, double_buffer):
    passed = True

    for cell_r, _ in enumerate(p_caches):
         for cell_n in neighborhood(cell_r):
            r_cache = p_caches[cell_r]
            n_cache = p_caches[cell_n]
            for addr_r, r in bram_enum(r_cache, double_buffer):
                if r is NULL:
                    continue
                for addr_n, n in bram_enum(n_cache, double_buffer):
                    if n is NULL or not n3l_np(r,n):
                        continue
                    reference = Position(cell = cell_r, addr = addr_r, r = r)
                    neighbor = Position(cell = cell_n, addr = addr_n, r = n)
                    pi = pair_ident(reference,neighbor)
                    if pi not in inputs:
                        print(f"expected {reference.origin()}, {neighbor.origin()}")
                        passed = False
                    inputs.remove(pi)

    for pi in inputs:
        reference, neighbor = pi_to_p(pi)
        print(f"unexpected {reference} {neighbor}")
        passed = False

    if not passed:
        exit(1)
