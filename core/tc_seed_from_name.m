function s = tc_seed_from_name(name)
% TC_SEED_FROM_NAME  Deterministic 32-bit seed from a scenario name (FNV-1a, uint64 arithmetic
% so no precision is lost).  Identical names give identical noise in every session.
  h = uint64(2166136261);
  for c = double(name(:))'
    h = bitxor(h, uint64(c));
    h = mod(h * uint64(16777619), uint64(4294967296));
  end
  s = double(h);
end
