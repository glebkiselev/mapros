from src.planner.scripts.mapcore.swm import Sign
from src.planner.scripts.mapcore.planning import mix_pairs
from copy import copy
from src.planner.scripts.mapcore.planning import PlanningTask as Task

signs = {}
obj_signifs = {}
obj_means = {}

def ground(problem, plagent, exp_signs = None):
    # ground I and They
    domain = problem.domain
    I_sign = Sign("I")
    obj_means[I_sign] = I_sign.add_meaning()
    obj_signifs[I_sign] = I_sign.add_significance()
    signs[I_sign.name] = I_sign
    They_sign = Sign("They")
    obj_means[They_sign] = They_sign.add_meaning()
    obj_signifs[They_sign] = They_sign.add_significance()
    signs[They_sign.name] = They_sign

    for type, stype in domain['types']:
        stype_sign = __add_sign(stype)
        stype_signif = stype_sign.add_significance()
        type_sign = __add_sign(type)
        connector = stype_signif.add_feature(obj_signifs[type_sign], zero_out=True)
        type_sign.add_out_significance(connector)

    for obj, type in problem.objects:
        obj_sign = __add_sign(obj)
        obj_means[obj_sign] = obj_sign.add_meaning()
        type_sign = signs[type]
        tp_signif = type_sign.add_significance()
        connector = tp_signif.add_feature(obj_signifs[obj_sign], zero_out=True)
        obj_sign.add_out_significance(connector)
        if obj_sign.name == plagent:
            connector = obj_signifs[obj_sign].add_feature(obj_signifs[I_sign], zero_out=True)
            I_sign.add_out_significance(connector)
            obj_means[obj_sign] = obj_sign.add_meaning()

    others = set()
    for id in range(1, len(signs['agent'].significances)+1):
        other_ag = signs['agent'].significances[id].get_signs()
        if signs[plagent] not in other_ag:
            others |= other_ag

    for subagent in others:
        if subagent.name != plagent:
            if not They_sign in subagent.significances[1].get_signs():
                signif = obj_signifs[They_sign]
                if signif.is_empty():
                    They_signif = signif
                else:
                    They_signif = They_sign.add_significance()
                connector = subagent.significances[1].add_feature(They_signif, zero_out=True)
                They_sign.add_out_significance(connector)
                obj_means[subagent] = subagent.add_meaning()

    for predicate in domain['predicates']:
        _ground_predicate(predicate.name, predicate.signature)

    for action in domain['actions']:
        _ground_action(action.name, action.parameters, action.preconditions, action.effect, problem.constraints, plagent)

    for task in domain['tasks']:
        __add_sign(task.name, False)

    methods = sorted(domain['methods'], key=lambda method: len(method.subtasks))
    for method in methods:
        __ground_method(method.parameters, method.subtasks, method.ordering, method.task, domain, 2)

    # Ground Init
    start = __add_sign('*start %s*' % str(problem.name), False)
    sit_im = start.add_image()
    for predicate in problem.init:
        pred_im = _ground_htn_predicate(predicate.name, predicate.signature, plagent)
        connector = sit_im.add_feature(pred_im)
        pred_im.sign.add_out_image(connector)
    sit_im.copy('image', 'meaning')

    # Ground htns to meanings
    goal = None
    subtasks = []
    for htn in problem.htns:
        htn_name = 'htn_' + str(problem.htns.index(htn))
        htn_sign = __add_sign(htn_name, False)
        htn_mean = htn_sign.add_meaning()
        for task in htn.ordering:
            subtask = htn.subtasks[task]
            cm = __ground_htn_subtask(subtask[0], subtask[1], domain)
            connector = htn_mean.add_feature(cm)
            cm.sign.add_out_meaning(connector)
        subtasks.append(htn_sign)

    from src.planner.scripts.mapmulti.grounding import signify_connection
    signify_connection(signs)

    return Task(problem.name, signs, start, goal, subtasks)


def __add_sign(sname, need_signif = True):
    if sname in signs:
        sign = signs[sname]
    else:
        sign = Sign(sname)
        signs[sname] = sign
        if need_signif:
            obj_signifs[sign] = sign.add_significance()
    return sign


def _ground_predicate(name, signature):
    pred_sign = __add_sign(name, False)
    # if more than 1 description to predicate
    pred_signif = pred_sign.add_significance()
    if len(signature):
        for signa in signature:
            if not signa[0].startswith('?'):
                right = '?' + signa[0]
            else:
                right = signa[0]
            new_obj = __add_sign(signa[1]+right, False)
            new_signif = new_obj.add_significance()
            old_type = signs[signa[1]]
            old_signif = old_type.add_significance()
            connector = new_signif.add_feature(old_signif, zero_out=True)
            old_type.add_out_significance(connector)
            connector = pred_signif.add_feature(new_signif)
            new_obj.add_out_significance(connector)
    return pred_signif

def _ground_action(name, parameters, preconditions, effect, constraints = None, plagent = None):
    action_sign = __add_sign(name, False)
    act_signif = action_sign.add_significance()
    def __update_significance(predicate, effect = False):
        pred_sign = signs[predicate[0]]
        if len(predicate[1]):
            pred_signs = []
            pred_signature = []
            for role in predicate[1]:
                role_name = list(filter(lambda x: x[0] == role, parameters))
                pred_signature.extend(role_name)
                if role_name[0][0].startswith('?'):
                    role_sig = role_name[0][1] + role_name[0][0]
                else:
                    role_sig = role_name[0][1] + '?' + role_name[0][0]
                try:
                    pred_signs.append(signs[role_sig])
                except KeyError:
                    role_sign = __add_sign(role_sig)
                    pred_signs.append(role_sign)

            flag = True
            pred_signif = None
            for _, signif in copy(pred_sign.significances).items():
                for pair in zip(pred_signs, signif.cause):
                    if not pair[0] in pair[1].get_signs():
                        break
                else:
                    flag = False
                    pred_signif = signif
                    break
            if flag:
                pred_signif = _ground_predicate(predicate[0], pred_signature)
            connector = act_signif.add_feature(pred_signif, effect=effect)
            pred_sign.add_out_significance(connector)

    for predicate in preconditions:
        __update_significance(predicate)
    for predicate in effect:
        __update_significance(predicate, effect=True)

    if constraints:
        preactivate_action_nonspecial(act_signif, constraints, plagent)
    return act_signif

def preactivate_action_nonspecial(act_signif, constraints, plagent):
    """
    This function activate nonspecial (common for all agents) actions
    """
    I_sign = signs['I']

    act_roles = {net[2].sign for net in act_signif.spread_down_activity('significance', 5)}

    for ag, constraint in constraints.items():
        predicates = set()
        signatures = []
        for predicate in constraint:
            predicates.add(predicate.name)
            signatures.append(predicate.signature)

        role_signifs = {}
        for signature in signatures:
            for signa in signature:
                roles = {cm.sign for cm in signs[signa].spread_up_activity_obj('significance', 2) if cm.sign in act_roles}
                for role in roles:
                    if signa == plagent:
                        signa = 'I'
                    role_signifs.setdefault(role, set()).add(signs[signa])

        variants = mix_pairs(role_signifs)

        for variant in variants:
            act_mean = act_signif.copy('significance', 'meaning')
            for role_sign, obj in variant.items():
                if obj not in obj_means:
                    obj_mean = obj_signifs[obj].copy('significance', 'meaning')
                else:
                    obj_mean = obj_means[obj]
                act_mean.replace('meaning', role_sign, obj_mean)
            if ag == plagent:
                connector = act_mean.add_feature(obj_means[I_sign])
                efconnector = act_mean.add_feature(obj_means[I_sign], effect=True)
                I_sign.add_out_meaning(connector)
            else:
                ag_mean = obj_means[signs[ag]]
                connector = act_mean.add_feature(ag_mean)
                efconnector = act_mean.add_feature(ag_mean, effect=True)
                signs[ag].add_out_meaning(connector)



def __ground_single_method(parameters, subtask, problem, depth):
    signifs = []
    actions = list(filter(lambda x: x.name ==subtask[0], problem['actions']))
    if len(actions):
        action = actions[0]
        subtask_parameters = []
        for param1 in subtask[1]:
            for param2 in parameters:
                if param1 == param2[0]:
                    subtask_parameters.append(param2)
        change = []
        #used = []
        if len(subtask_parameters) == len(action.parameters):
            for pair in zip(subtask_parameters, action.parameters):
                if pair[0] != pair[1]:
                    change.append((pair[1][0], pair[0][0]))
        if change:
            preconditions = []
            effect = []
            changed = []
            for predicate in action.preconditions:
                if predicate in changed:
                    continue
                new_predicate = [predicate[0], []]
                for signa in predicate[1]:
                    for pair in change:
                        if signa == pair[0]:
                            new_predicate[1].append(pair[1])
                            break
                    else:
                        new_predicate[1].append(signa)
                preconditions.append(new_predicate)
                changed.append(predicate)
            changed = []
            for predicate in action.effect:
                if predicate in changed:
                    continue
                new_predicate = [predicate[0], []]
                for signa in predicate[1]:
                    for pair in change:
                        if signa == pair[0]:
                            new_predicate[1].append(pair[1])
                            break
                    else:
                        new_predicate[1].append(signa)
                effect.append(new_predicate)
                changed.append(predicate)
            signif = _ground_action(subtask[0], parameters, preconditions, effect)
        else:
            signif = signs[action.name].significances[1]
        signifs.append(signif)
    else:
        if depth <= 1:
            return None
            #raise Exception("Can not ground method with low depth")
        methods = list(filter(lambda x: x.task == subtask[0], problem['methods']))
        for old_method in methods:
            change = []
            for param1 in subtask[1]:
                for param2 in old_method.task_parameters:
                    if subtask[1].index(param1) == old_method.task_parameters.index(param2) and param1 != param2:
                        change.append((param2, param1))
            new_params = []
            if change:
                for param2 in old_method.parameters:
                    for param1 in change:
                        if param2[0] == param1[0]:
                            new_params.append((param1[1], param2[1]))
                            break
                        elif param2[0] == param1[1]:
                            new_params.append((param1[0], param2[1]))
                            break
                        else:
                            new_params.append(param2)
                            break
                    else:
                        new_params.append(param2)
            else:
                signif = signs[subtask[0]].significances[1]
                signifs.append(signif)
                return signifs

            old_subtasks = {}
            for tnum, stask in old_method.subtasks.items():
                stask_signa = []
                for param1 in stask[1]:
                    for param2 in change:
                        if param1 == param2[0]:
                            stask_signa.append(param2[1])
                            break
                        elif param1 == param2[1]:
                            stask_signa.append(param2[0])
                            break
                    else:
                        stask_signa.append(param1)
                old_subtasks[tnum] = (stask[0], stask_signa)
            old_t_param = []
            for param1 in old_method.task_parameters:
                for param2 in change:
                    if param1 == param2[0]:
                        old_t_param.append(param2[1])
                        break
                    elif param1 == param2[1]:
                        old_t_param.append(param2[0])
                        break
                else:
                    old_t_param.append(param1)
            signif = __ground_method(new_params, old_subtasks, old_method.ordering, old_method.task, problem, depth - 1)
            if signif:
                signifs.extend(signif)
    return signifs

def __ground_method(parameters, subtasks, ordering, task, problem, depth):
    task = signs[task]
    stasks = {}
    task_signifs = []
    for tasknum, subtask in subtasks.items():
        signifs = __ground_single_method(parameters, subtask, problem, depth)
        if signifs:
            stasks[tasknum] = signifs
        else:
            return None
    if len(stasks) == 1:
        signifs = stasks['task0']
        for signif in signifs:
            task_signif = task.add_significance()
            connector = task_signif.add_feature(signif)
            signif.sign.add_out_significance(connector)
            task_signifs.append(task_signif)
    else:
        variants = mix_pairs(stasks)
        for variant in variants:
            task_signif = task.add_significance()
            for order in ordering:
                signif = variant[order]
                connector = task_signif.add_feature(signif)
                signif.sign.add_out_significance(connector)
            task_signifs.append(task_signif)
    return task_signifs


def __ground_htn_subtask(name, args, problem):
    methods  = [method for method in problem['methods'] if method.task == name]
    change = {}
    fin_meth = None
    tparams = []
    for method in methods:
        for e1 in method.task_parameters:
            for e2 in method.parameters:
                if e1 == e2[0]:
                    tparams.append(e2)
                    break
        if len(args) == len(tparams):
            cms = [signs[arg].meanings[1] for arg in args]
            for param in tparams:
                change[signs[param[1]+param[0]]] = cms[tparams.index(param)]
            fin_meth = method
            break
        else:
            tparams = []
            change = {}
    chparams = [el[0] for el in tparams]
    fin_meth_mean = signs[fin_meth.task].add_meaning()
    acts = []

    #TODO Choose the correct significance when its not alone

    for event in signs[fin_meth.task].significances[1].cause:
        for connector in event.coincidences:
            cm = getattr(connector.out_sign, 'significances')[connector.out_index]
            acts.append(cm)
    htn_methods = {}
    for stask, parameters in fin_meth.subtasks.items():
        bothel = [el for el in parameters[1] if el in chparams]
        if not bothel:
            act = acts[fin_meth.ordering.index(stask)]
            htn_method = act.copy('significance', 'meaning')
        else:
            act = acts[fin_meth.ordering.index(stask)]
            htn_method = act.copy('significance', 'meaning')
        for sign, cm in change.items():
            htn_method.replace('meaning', sign, cm)
        htn_methods[stask] = htn_method
    for task in fin_meth.ordering:
        htn_method = htn_methods[task]
        connector = fin_meth_mean.add_feature(htn_method)
        htn_method.sign.add_out_meaning(connector)

    return fin_meth_mean


def _ground_htn_predicate(name, signature, plagent):
    pred_sign = signs[name]
    pred_im = pred_sign.add_image()
    for element in signature:
        if element != plagent:
            el_sign = signs[element]
        else:
            el_sign = signs["I"]
        el_image = el_sign.add_image()
        con = pred_im.add_feature(el_image)
        el_sign.add_out_image(con)
    return pred_im


def _create_methods_tree(domain):
    import collections
    """
    Function that creates methods depth tree. Its need for prevent recursive including Causal Matrix to itself.
    """
    tree = {}
    methods = sorted(domain['methods'], key=lambda method: len(method.subtasks))
    methods_names = {method.task for method in methods}
    actions_names = {action.name for action in domain['actions']}
    for method in methods:
        subtasks = {task[0] for task in method.subtasks.values()}
        if subtasks.isdisjoint(actions_names):
            level = 3
        elif subtasks & actions_names and subtasks & methods_names:
            level = 2
        elif subtasks <= actions_names:
            level = 1
        else:
            level = 0
            print('Wrong method description')
        tree.setdefault(level, []).append(method)

    tree = collections.OrderedDict(sorted(tree.items()))

    return tree





