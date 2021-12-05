function [vert, fac, vf, xeva, yeva, zeva] = gpis_runner(path)
    [~, vert, fac, vf, xeva, yeva, zeva] = evalc('gpis(path);');
end