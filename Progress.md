## Traj. tracking controller 
First the desired velocity is shown

$$ {{v}_d} = {\dot {{\sigma}} _r} + {K_p}\tanh({ {e}_p} ) $$

Then provide a desired acceleration

$$ \dot { v} = {{\dot { v}}_d} + {K_v}K_p^{ - 1}{\tanh({ e_p})} + {K_v}{\tanh({ e_v})} $$

Then the problem is how to generate ${{\dot { v}}_d}$, 
of course we can compute them analytically, but a differential tracker is much better
for convenience.

### Vertical frame

The desire used in the vertical frame can be described as 

$$ {\psi_d}  = \arctan 2( {{a_{xd}},{a_{yd}}} ) $$

$$ {}^V{{\dot v} _ {xd}}  = \sqrt{a_{xd}^2 + a_{yd}^2} $$ 

$$ {}^V{{\dot v} _ {zd}}  = {a_{zd}} $$

the nominal control inputs of the positional subsystem are given as

$$ f_{\rm flap}^2 = k_{\rm tf}^{ - 1}m\sqrt {\dot v_{cx}^2 + \dot v_{cz}^2} $$ 

$$ {\Gamma _ {xd}} = {{{{\dot v}_ {cx}}} \mathord{\left/
 {\vphantom {{{{\dot v}_ {cx}}} {\sqrt {\dot v_{cx}^2 + \dot v_{cz}^2} }}} \right.
 \kern-\nulldelimiterspace} {\sqrt {\dot v_{cx}^2 + \dot v_{cz}^2} }} $$ 
 
$$ {\Gamma _ {zd}} = {{{{\dot v}_ {cz}}} \mathord{\left/
 {\vphantom {{{{\dot v}_ {cz}}} {\sqrt {\dot v_{cx}^2 + \dot v_{cz}^2} }}} \right.
 \kern-\nulldelimiterspace} {\sqrt {\dot v_{cx}^2 + \dot v_{cz}^2} }} $$ 
 
