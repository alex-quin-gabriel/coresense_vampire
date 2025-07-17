
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "coresense_msgs/action/query_reasoner.hpp"
#include "coresense_msgs/srv/add_knowledge.hpp"
#include "coresense_msgs/srv/remove_knowledge.hpp"
#include "coresense_msgs/srv/pop_knowledge.hpp"
#include "coresense_msgs/srv/list_knowledge.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include <iostream>
#include <ostream>
#include <fstream>
#include <csignal>

#include "Vampire.hpp"



namespace vampire_node_cpp {
/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */
class VampireNode : public rclcpp::Node
{
  public:
    using QueryReasoner = coresense_msgs::action::QueryReasoner;
    using GoalHandleQueryReasoner = rclcpp_action::ServerGoalHandle<QueryReasoner>;
    VampireNode()
    : Node("vampire_node")
    {
      Vampire::init();

      // Services
      list_knowledge_service = create_service<coresense_msgs::srv::ListKnowledge>("list_knowledge", std::bind(&VampireNode::list_knowledge, this, std::placeholders::_1, std::placeholders::_2));
      add_knowledge_service = create_service<coresense_msgs::srv::AddKnowledge>("add_knowledge", std::bind(&VampireNode::add_knowledge, this, std::placeholders::_1, std::placeholders::_2));
      remove_knowledge_service = create_service<coresense_msgs::srv::RemoveKnowledge>("remove_knowledge", std::bind(&VampireNode::remove_knowledge, this, std::placeholders::_1, std::placeholders::_2));
      pop_knowledge_service = create_service<coresense_msgs::srv::PopKnowledge>("pop_knowledge", std::bind(&VampireNode::pop_knowledge, this, std::placeholders::_1, std::placeholders::_2));

      // Actions
      this->action_server_ = rclcpp_action::create_server<QueryReasoner>(
        this,
        "vampire",
        std::bind(&VampireNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&VampireNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&VampireNode::handle_accepted, this, std::placeholders::_1));
    }

  private:

    rclcpp::Service<coresense_msgs::srv::ListKnowledge>::SharedPtr list_knowledge_service;
    rclcpp::Service<coresense_msgs::srv::AddKnowledge>::SharedPtr add_knowledge_service;
    rclcpp::Service<coresense_msgs::srv::RemoveKnowledge>::SharedPtr remove_knowledge_service;
    rclcpp::Service<coresense_msgs::srv::PopKnowledge>::SharedPtr pop_knowledge_service;

    rclcpp_action::Server<QueryReasoner>::SharedPtr action_server_;

    void add_knowledge(const std::shared_ptr<coresense_msgs::srv::AddKnowledge::Request>  request,
                             std::shared_ptr<coresense_msgs::srv::AddKnowledge::Response> response)
    {
      RCLCPP_INFO(this->get_logger(), "Received theory %s ", request->id.c_str());
      RCLCPP_INFO(this->get_logger(), "With content %s ", request->tptp.c_str());
      response->success = Vampire::loadTPTP(request->id.c_str(), request->tptp.c_str());
    }

    void remove_knowledge(const std::shared_ptr<coresense_msgs::srv::RemoveKnowledge::Request>  request,
                                std::shared_ptr<coresense_msgs::srv::RemoveKnowledge::Response> response)
    {
      // TODO add api call to remove by id
      //requires more fiddling
	    //delete request->knowledge;
      //provide response response->success = true;
    }

    void pop_knowledge(const std::shared_ptr<coresense_msgs::srv::PopKnowledge::Request>  request,
                                std::shared_ptr<coresense_msgs::srv::PopKnowledge::Response> response)
    {
      response->success = Vampire::popTheories(request->count);
    }

    void list_knowledge(const std::shared_ptr<coresense_msgs::srv::ListKnowledge::Request>  request,
                              std::shared_ptr<coresense_msgs::srv::ListKnowledge::Response> response)
    {
      response->tptp = Vampire::listTheories();
    }

    rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID & uuid,
      std::shared_ptr<const QueryReasoner::Goal> goal)
    {
      RCLCPP_INFO(this->get_logger(), "Received query: %s", goal->query.c_str());
      RCLCPP_INFO(this->get_logger(), "Received config: %s", goal->configuration.c_str());
      //RCLCPP_INFO(this->get_logger(), "Prover status is %d", Vampire::getStatus());

      if (Vampire::ProverStatus::READY == Vampire::getStatus())
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      else
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleQueryReasoner> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
      (void)goal_handle;
      Vampire::stopProver();
      return rclcpp_action::CancelResponse::ACCEPT;
    }
    
    void handle_accepted(const std::shared_ptr<GoalHandleQueryReasoner> goal_handle)
    {
      const auto goal = goal_handle->get_goal();
      RCLCPP_INFO(this->get_logger(), "Running Vampire");
      Vampire::runProver(goal->query.c_str(), goal->configuration.c_str());
      RCLCPP_INFO(this->get_logger(), "Vampire::runProver() returned, starting watcher.");
      std::thread{std::bind(&VampireNode::check_on_goal, this, std::placeholders::_1), goal_handle}.detach();
      rclcpp::Rate loop_rate(0.1);
	    loop_rate.sleep();
      auto status = Vampire::getStatus();
      RCLCPP_INFO(this->get_logger(), "status %d", status);
      RCLCPP_INFO(this->get_logger(), "stop returns %b", Vampire::stopProver());
      status = Vampire::getStatus();
      RCLCPP_INFO(this->get_logger(), "status %d", status);
    }

    void check_on_goal(const std::shared_ptr<GoalHandleQueryReasoner> goal_handle)
    {
      RCLCPP_INFO(this->get_logger(), "Executing goal");
      rclcpp::Rate loop_rate(1);
      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<QueryReasoner::Feedback>();
      auto status = Vampire::getStatus();
      while (Vampire::ProverStatus::RUNNING == status) {
	      feedback->status = "running";
        goal_handle->publish_feedback(feedback);
	      loop_rate.sleep();
        status = Vampire::getStatus();
      }
      auto result = std::make_shared<QueryReasoner::Result>();

      switch (status) {
        case (Vampire::ProverStatus::SUCCEEDED):
	        result->result = Vampire::getSolution();
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal succeeded");
	        return;
        case (Vampire::ProverStatus::FAILED):
          RCLCPP_WARN(this->get_logger(), "Vampire failed to find a proof.");
	        return;
        case (Vampire::ProverStatus::SIGNALLED):
          RCLCPP_WARN(this->get_logger(), "Vampire subprocess ended with signal %d.", Vampire::lastSignal());
	        return;
        case (Vampire::ProverStatus::ERROR):
          RCLCPP_WARN(this->get_logger(), "Vampire prover returned error.");
	        return;
	      default:
	        goal_handle->canceled(0);
          RCLCPP_ERROR(this->get_logger(), "Unknown Result, bug the maintainer.");
	        return;
      }
    }

    void check_conjecture_callback(const std_msgs::msg::String & msg) const
    {
      // takes a set of axioms (formulas) or clauses and a conjecture (formula or set of clauses)
      // outputs proof
      // method: negation of conjecture and checking for unsatisfiability
      // internally converts to CNF (Clause Normal Form (Conjunction of Disjunctions)) and runs saturation algorithm
      // i.e. computes a closure and applies resolution+superposition
    }

  };
}
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  RCLCPP_INFO(rclcpp::get_logger("Vampire"), "pid: %i", getpid());
  rclcpp::spin(std::make_shared<vampire_node_cpp::VampireNode>());
  rclcpp::shutdown();
  return 0;
}
