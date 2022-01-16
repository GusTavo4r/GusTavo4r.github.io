function [ bestindividual, bestfit ] = best( pop, value)
bestfit = max(value);
i = find(value==bestfit,1);
bestindividual = pop(i,:);
end