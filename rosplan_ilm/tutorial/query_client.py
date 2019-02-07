#!/usr/bin/env python

import sys
import rospy
from rosplan_knowledge_msgs.srv import *
from rosplan_knowledge_msgs.msg import *

query = []

def call_service():
    print "Waiting for service"
    rospy.wait_for_service('/rosplan_knowledge_base/query_state')
    try:
        print "Calling Service"
        query_proxy = rospy.ServiceProxy('rosplan_knowledge_base/query_state', KnowledgeQueryService)
        resp1 = query_proxy(query)
        print "Response is:", resp1.results
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":

    # QUERY 1 (robot_at kenny wp0)
    query1 = KnowledgeItem()
    query1.knowledge_type = KnowledgeItem.FACT
    query1.attribute_name = "robot_at"
    query1.values.append(diagnostic_msgs.msg.KeyValue("v", "kenny"))
    query1.values.append(diagnostic_msgs.msg.KeyValue("wp", "wp0"))
    query.append(query1)

    # QUERY 2 (robot_at kenny wp3)
    query2 = KnowledgeItem()
    query2.knowledge_type = KnowledgeItem.FACT
    query2.attribute_name = "robot_at"
    query2.values.append(diagnostic_msgs.msg.KeyValue("v", "kenny"))
    query2.values.append(diagnostic_msgs.msg.KeyValue("wp", "wp3"))
    query.append(query2)

        # QUERY 3 (robot_at kenny wp3)
    query3 = KnowledgeItem()
    query3.knowledge_type = KnowledgeItem.INEQUALITY
    query3.ineq.comparison_type = DomainInequality.GREATER
    query3.ineq.grounded = True

    token1 = ExprBase()
    token1.expr_type = ExprBase.CONSTANT
    token1.constant = 125
    query3.ineq.LHS.tokens.append(token1)

    token2 = ExprBase()
    token2.expr_type = ExprBase.OPERATOR
    token2.op = ExprBase.ADD

    token3 = ExprBase()
    token3.expr_type = ExprBase.CONSTANT
    token3.constant = 5

    token4 = ExprBase()
    token4.expr_type = ExprBase.FUNCTION
    token4.function.name = "energy"
    token4.function.typed_parameters.append(diagnostic_msgs.msg.KeyValue("v", "kenny"))

    query3.ineq.RHS.tokens.append(token2)
    query3.ineq.RHS.tokens.append(token3)
    query3.ineq.RHS.tokens.append(token4)

    query.append(query3)
    
    call_service()
    sys.exit(1)