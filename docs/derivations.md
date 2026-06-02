Bayes rule conditioned on $c$: 
$$p(a \mid b, c) = \frac{p(b \mid a, c)p(a \mid c)}{p(b \mid c)}$$
So: 
$$p(x_{1:t} \mid z_{1:t}, u_{1:t}) = \frac{p(z_{1:t} \mid x_{1:t}, u_{1:t})p(x_{1:t} \mid u_{1:t})}{p(z_{1:t} \mid u_{1:t})}$$



Rules of total probability: 
$$p(a) = \int p(a, b) \, db$$
(To find the total probability of $A$, look at the probability of $A$ under every possible scenario of $B$, weight them by how likely $B$ is, and add them all up.).

If we conditioned on $c$, we have: 
$$p(a \mid c) = \int p(a \mid b, c)p(b \mid c) \, db$$
which means that:
$$p(z_{1:t} \mid u_{1:t}) = \int p(z_{1:t} \mid x_{1:t}, u_{1:t})p(x_{1:t} \mid u_{1:t}) \, dx_{1:t}$$
