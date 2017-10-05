%Should it come up again, this is justification for why our theta error is
%less smooth than our x or y error

x = 0:0.1:100
%Create something really not smooth and plot
y = sin(x) + 0.5*sin(10*x)
plot(x, y)

%Integrate y and plot on a second window. Integral is less squiggly
i = zeros(size(x))

for z = 1:size(x, 2)
    for w = 1:z
        i(z) = i(z) + y(w)*0.1;
    end
    z
end

figure
plot(x, i)