import sympy as sp
from sympy import cos, sin, Matrix, simplify
from sympy.printing.cxx import CXX11CodePrinter
from sympy.utilities.codegen import codegen


def main():
    
    del_phi_j, l_j, theta_j = sp.symbols('del_phi_j l_j theta_j')
    J11 = ((cos(del_phi_j)*(cos(theta_j)-1))/((theta_j/l_j)**2))
    J21 = ((sin(del_phi_j)*(cos(theta_j)-1))/((theta_j/l_j)**2))
    J31 = -(sin(theta_j)-theta_j)/((theta_j/l_j)**2)
    J41 = -l_j*sin(del_phi_j)
    J43 =  -((theta_j/l_j)**2)*sin(del_phi_j)
    J51 = l_j*cos(del_phi_j)
    J53 = ((theta_j/l_j)**2)*cos(del_phi_j)
    J = Matrix([
        [J11, 0, 0],
        [J21, 0, 0],
        [J31, 0, 1],
        [J41, 0, J43],
        [J51, 0, J53],
        [0, 1, 0]
    ])
    # J = J.subs([(del_phi_j,0), (l_j,1)])
    # J = simplify(J, inverse=True)
    
    # printer = CXX11CodePrinter()
    # Jacobian = sp.MatrixSymbol('Jacobian', 6, 3)
    [(cf, cs), (hf, hs)] = codegen(('main', J), language='c')
    file_c = open(cf, 'w')
    file_c.write(cs)
    file_h = open(hf, 'w')
    file_h.write(hs)


    # print(printer.doprint(J))
    # code = (printer.doprint(J, assign_to=Jacobian))
    return J
    
if __name__ == "__main__":
    J = main()
