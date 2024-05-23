import os
import networkx as _nx
from gr1py.form import gr1c, util
from tulip import spec, synth
# import tulip.transys.export.machine2scxml as _scxml

def _ast2expr(ast):
    if isinstance(ast, tuple):
        output = ''
        if len(ast) == 2:
            output += '(' + _ast2expr(ast[0]) + _ast2expr(ast[1]) + ')'
        elif len(ast) == 3:
            output += '(' + _ast2expr(ast[1]) + _ast2expr(ast[0]) + _ast2expr(ast[2]) + ')'
        else:
            raise ValueError('Unexpected 4-tuple in AST')
    else:
        if ast == '=':
            output = '=='
        else:
            output = str(ast)
    return output

def _get_expr(asd:dict, key:str):
    if not key in asd or asd[key] == '':
        return ["TRUE"]
    else:
        ast = asd[key]
        if isinstance(ast, str):
            return [ast]
        elif isinstance(ast, tuple):
            return [_ast2expr(ast)]
        elif isinstance(ast, list):
            return [_ast2expr(a) for a in ast]

def load_GRSpec(filename:str) -> spec.GRSpec:
    f = open(filename, 'r')
    data = f.read() 
    f.close()

    asd = gr1c.parse(data)

    # get gr1 specs
    env_vars = {v[0] for v in asd['ENV']}
    sys_vars = {v[0] for v in asd['SYS']}
    env_init = _get_expr(asd, 'ENVINIT')
    sys_init = _get_expr(asd, 'SYSINIT')
    env_safety = _get_expr(asd, 'ENVTRANS')
    sys_safety = _get_expr(asd, 'SYSTRANS')
    env_prog = _get_expr(asd, 'ENVGOAL')
    sys_prog = _get_expr(asd, 'SYSGOAL')

    specs = spec.GRSpec(env_vars=env_vars, sys_vars=sys_vars, env_init=env_init, sys_init=sys_init, env_safety=env_safety, sys_safety=sys_safety, env_prog=env_prog, sys_prog=sys_prog)
    specs.moore = False
    specs.plus_one = False
    specs.qinit = r'\A \E'

    return specs


# def dump_strategy_class(path, name, strat):
#     dumpsmach.write_python_case(path + '/' + name + "ctrl.py", strat, classname=name.capitalize() + 'Ctrl')

# def dump_strategy_xml(path, name, strat):
#     s = _scxml.mealy2scxml(strat)

#     # dump to file
#     with open(path + '/' + name + '.xml', 'w') as f:
#         f.write(s)
#         f.close()



if __name__ == "__main__":

    specs = load_GRSpec("/home/dyq/catkin_ws/src/gr1strategy/specs/demo_gr1.spc")
    
    # Mealy
    ctrl = synth.synthesize(specs)
    print(ctrl)
    # print(ctrl.inputs)
    # print(ctrl.outputs)
    # next_state, output_props = ctrl.reaction('Sinit', {})
    # print(next_state, output_props)

    