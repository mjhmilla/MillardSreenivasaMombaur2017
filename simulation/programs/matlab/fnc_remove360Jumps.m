function matOut = fnc_remove360Jumps(matIn)
[rows,cols] = size(matIn);
for j=1:cols
    x = matIn(:,j);
    d = diff(x);
    q = abs(d) > 150;       % assume that changes of 270 deg never happen, normally
    d(q) = d(q) - sign(d(q))*360;   % add or subtract 360 deg
    d(isnan(d))=0;
    matOut(:,j) = cumsum([x(1,:); d]);
end