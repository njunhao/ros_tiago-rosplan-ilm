import sys

def main(argv):
    #command = "python3 " + parser_path + " write_plan " + plan_path + "\"" + action + "\"";
    
    if str(sys.argv[1]) == "write_plan":
        write_plan(sys.argv[2], sys.argv[3])    # args: file path, plan
    elif str(sys.argv[1]) == "write_state_objects":
        write_state_objects(sys.argv[2], sys.argv[3], sys.argv[4])    # args: target file path, target (ILM) file path, PDDL string
    elif str(sys.argv[1]) == "write_pddl":
        write_pddl(sys.argv[2], sys.argv[3], sys.argv[4])        # args: source (ILM) path, target PDDL domain path, target PDDL problem path

    #Open the file back and read the contents
    #f=open("guru99.txt", "r")
    #   if f.mode == 'r': 
    #     contents =f.read()
    #     print contents
    #or, readlines reads the individual line into a list
    #fl =f.readlines()
    #for x in fl:
    #print x
    
    
def write_plan(file, action):
    start_time = "0.000"
    duration = "0.000"
    f = open(file,"w+")
    start_id = action.find("(")
    action_name = action[:start_id]
    terms = []
    while (start_id < len(action)):
        end_id = action.find(",", start_id+1)
        if (end_id > 0):
            terms += [action[start_id+1:end_id]]
            start_id = end_id
        else:
            terms += [action[start_id+1:-1]]
            break
    # action from rexd: moveCar(l12, l31)
    # Plan for ROSPlan
    # 0.000: (undock kenny wp1)  [10.000]
    line = start_time + ": (" + action_name + " "
    for x in range(0, len(terms)):
        line += terms[x].lstrip().rstrip()
        if x < len(terms)-1:
            line += " "
        else:
            line += ") [" + duration + "]"
    print(line)
    f.write(line)
    f.close()
    
    
def write_state_objects(state_objects_file, pddl_problem_file, pddl_problem_string):
    # define constants
    DOMAIN_STRING = "(:domain"
    OBJECT_STRING = "(:objects"
    INIT_STRING = "(:init"
    GOAL_STRING = "(:goal"
    METRIC_STRING = "(:metric"

    # get dictionary with keys as type of objects and values as the objects
    start_id = pddl_problem_string.find(OBJECT_STRING) + len(OBJECT_STRING)
    end_id = pddl_problem_string.find(INIT_STRING) - 1    # -1 to remove )
    objects_string = pddl_problem_string[start_id:end_id].lstrip().split(" ")
    objects = []
    typed_objects = {}
    is_type = False
    for object in objects_string:
        if len(object) == 0:
            continue
        elif object == "-":
            is_type = True  # next object is typed for all previous objects
        elif is_type:
            typed_objects.update({object: objects})
            objects = []
            is_type = False
        else:
            objects += [object]
            
    f1 = open(state_objects_file,"w+")
    
    # write state string
    # e.g. -hasspare() notFlattire() road(l11 - location l21 - location) road(l12 - location l11 - location) road(l12 - location l22 - location) road(l13 - location l12 - location) road(l13 - location l22 - location) road(l21 - location l31 - location) road(l22 - location l21 - location) road(l22 - location l31 - location) spareIn(l11 - location) spareIn(l12 - location) spareIn(l21 - location) vehicleAt(l13 - location)
    start_id = pddl_problem_string.find(INIT_STRING) + len(INIT_STRING)
    end_id = pddl_problem_string.find(GOAL_STRING) - 1    # -1 to remove )
    predicates = pddl_problem_string[start_id:end_id].lstrip().replace('(', ')').split(')')  # replace ( with ), then split using )
    predicates = [predicate for predicate in predicates if len(predicate.lstrip())>1]  # clean up, remove blank spaces
    state_string = ""
    for predicate in predicates:
        objects = predicate.split(" ")
        state_string += objects[0] + "("   # predicate name
        for object in objects[1:]:
            for key, value in typed_objects.items():
                if object in value:
                    typed = key
                    break
            state_string += object + " - " + typed + " "
        state_string = state_string.rstrip() + ") "
    f1.write(state_string+"\n")
    
    # write objects string (e.g. (:objects wp0 wp1 wp2 wp3 wp4 - waypoint kenny - robot ))
    objects_string = OBJECT_STRING + " "
    for typed in typed_objects.items():
        for object in typed[1]:
            objects_string += object + " "
        objects_string += "- " + typed[0] + " "
    objects_string += ")"
    f1.write(objects_string)
    f1.close()

    # re-write PDDL problem.pddl for ILM to reload scenario (when new objects are added or goals have changed in ROSPlan)
    f2 = open(pddl_problem_file,"w+")
    start_id = 0
    end_id = pddl_problem_string.find(DOMAIN_STRING)
    string = pddl_problem_string[start_id:end_id]           # (define (problem task)
    f2.write(string+"\n")
    
    start_id = end_id
    end_id = pddl_problem_string.find(OBJECT_STRING)
    string = pddl_problem_string[start_id:end_id]           # (:domain turtlebot)
    f2.write(string+"\n")

    start_id = end_id
    end_id = pddl_problem_string.find(INIT_STRING)
    string = pddl_problem_string[start_id:end_id]           # (:objects wp0 wp1 wp2 wp3 wp4 - waypoint kenny - robot)
    f2.write(string+"\n")

    start_id = end_id
    end_id = pddl_problem_string.find(GOAL_STRING)
    string = pddl_problem_string[start_id:end_id]           # (:init ...
    f2.write(string+"\n")

    start_id = end_id
    end_id = pddl_problem_string.find(METRIC_STRING)
    string = pddl_problem_string[start_id:end_id]           # (:goal ...
    f2.write(string+"\n")

    f2.write("(:goal-reward 1)\n(:metric maximize (reward))\n)")

    f2.close()
    

def write_pddl(source_path, domain_path, problem_path):
#    with open(source_path + "/" + "rexd_config.cfg","r") as f1:
#        for line in f1:
#            if "prior_domain" in line:
#                pos = line.find("#")   # ignore comment
#                if "param" in line[:pos]:
#                    source_domain = source_path + "/" + "domain_param.pddl"
#                elif "approx" in line[:pos]:
#                    source_domain = source_path + "/" + "domain_approx.pddl"
#                else:
#                    source_domain = source_path + "/" + "domain_ground_truth.pddl"
#                break
    
    source_domain = source_path + "/" + "domain_ground_truth.pddl"   # always use ground truth domain in ROSPlan
    with open(domain_path, 'w') as f2:
        for line in open(source_domain):
            if 'requirements' in line and ':rewards' in line:
                new_line = line.replace(':rewards', '')  # remove this requirement as ROSPlan cannot accept it
                f2.write(new_line)
            elif ';;' not in line:                       # do not insert reliability stuff, else will get assertion error in rexd (grounding.empty())
                f2.write(line)

    with open(problem_path, 'w') as f3:
        for line in open(source_path + "/" + "problem_current.pddl"):
            if ':goal-reward' in line or ':metric' in line:
                # do not add these lines
                pass
            else:
                f3.write(line)

if __name__== "__main__":
    main(sys.argv)