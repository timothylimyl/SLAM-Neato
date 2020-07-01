function idx = resampleSystematic(logW)

N = length(logW);

idx = zeros(N, 1);
u = [linspace(0, 1 - 1/N, N) + rand/N, 1];
s = cumsum(exp(logW));  % Not worried about underflow here
i = 1;
j = 1;
while i <= N
    if u(i) < s(j)
        idx(i) = j;
        i = i + 1;
    else
        j = j + 1;
    end
end
