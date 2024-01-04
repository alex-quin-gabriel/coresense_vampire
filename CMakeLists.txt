cmake_minimum_required(VERSION 3.8)
project(coresense_vampire)

add_definitions(-w)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
SET(COMPILE_TESTS OFF)

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/Lib
  ${CMAKE_SOURCE_DIR}/src/Kernel
  ${CMAKE_SOURCE_DIR}/src/Shell )

link_directories(${CMAKE_SOURCE_DIR}/src)
set(VAMPIRE_MINISAT_SOURCES
    ${CMAKE_SOURCE_DIR}/src/Minisat/core/Solver.cc
    ${CMAKE_SOURCE_DIR}/src/Minisat/simp/SimpSolver.cc
    ${CMAKE_SOURCE_DIR}/src/Minisat/utils/Options.cc
    ${CMAKE_SOURCE_DIR}/src/Minisat/utils/System.cc
    ${CMAKE_SOURCE_DIR}/src/SAT/MinisatInterfacing.cpp
    ${CMAKE_SOURCE_DIR}/src/SAT/MinisatInterfacingNewSimp.cpp
    ${CMAKE_SOURCE_DIR}/src/Minisat/core/Dimacs.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/core/Solver.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/core/SolverTypes.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/mtl/Alg.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/mtl/Alloc.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/mtl/Heap.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/mtl/IntMap.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/mtl/IntTypes.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/mtl/Map.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/mtl/Queue.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/mtl/Rnd.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/mtl/Sort.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/mtl/Vec.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/mtl/XAlloc.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/simp/SimpSolver.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/utils/Options.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/utils/ParseUtils.h
    ${CMAKE_SOURCE_DIR}/src/Minisat/utils/System.h
    )
source_group(minisat_source_files FILES ${VAMPIRE_MINISAT_SOURCES})

set(VAMPIRE_DEBUG_SOURCES
    ${CMAKE_SOURCE_DIR}/src/Debug/Assertion.cpp
    ${CMAKE_SOURCE_DIR}/src/Debug/RuntimeStatistics.cpp
    ${CMAKE_SOURCE_DIR}/src/Debug/Tracer.cpp
    ${CMAKE_SOURCE_DIR}/src/Debug/Assertion.hpp
    ${CMAKE_SOURCE_DIR}/src/Debug/RuntimeStatistics.hpp
    ${CMAKE_SOURCE_DIR}/src/Debug/Tracer.hpp
    ${CMAKE_SOURCE_DIR}/src/Debug/Output.hpp
    )
source_group(debug_source_files FILES ${VAMPIRE_DEBUG_SOURCES})

set(VAMPIRE_LIB_SOURCES
    ${CMAKE_SOURCE_DIR}/src/Lib/Allocator.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/DHMap.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Environment.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Event.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Exception.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Int.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/IntNameTable.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/IntUnionFind.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/MemoryLeak.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/MultiCounter.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/NameArray.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Random.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/StringUtils.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/System.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Timer.cpp

    ${CMAKE_SOURCE_DIR}/src/Lib/Allocator.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Array.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/ArrayMap.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Backtrackable.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/BacktrackIterators.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/BinaryHeap.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/BitUtils.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Comparison.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Counter.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/DArray.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Deque.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/DHMap.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/DHMultiset.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/DHSet.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/DynamicHeap.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Environment.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Event.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Exception.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/fdstream.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/FreshnessGuard.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Hash.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Int.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/IntNameTable.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/IntUnionFind.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/InverseLookup.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/List.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Map.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/MaybeBool.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/MemoryLeak.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Metaiterators.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/MultiCounter.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/NameArray.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Numbering.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/PairUtils.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Portability.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Random.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/RatioKeeper.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Recycled.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Reflection.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/SafeRecursion.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/ScopedLet.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/ScopedPtr.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Set.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/SharedSet.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/SkipList.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/SmartPtr.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Sort.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Stack.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/STLAllocator.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/StringUtils.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/System.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Timer.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/TriangularArray.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Vector.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/VirtualIterator.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/VString.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/STL.hpp
    )
source_group(lib_source_files FILES ${VAMPIRE_LIB_SOURCES})

set(VAMPIRE_LIB_SYS_SOURCES
    ${CMAKE_SOURCE_DIR}/src/Lib/Sys/Multiprocessing.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Sys/Semaphore.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Sys/SyncPipe.cpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Sys/Multiprocessing.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Sys/Semaphore.hpp
    ${CMAKE_SOURCE_DIR}/src/Lib/Sys/SyncPipe.hpp
    )
source_group(lib_sys_source_files FILES ${VAMPIRE_LIB_SYS_SOURCES})

set(VAMPIRE_KERNEL_SOURCES
    ${CMAKE_SOURCE_DIR}/src/Kernel/Clause.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/ClauseQueue.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/ColorHelper.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/ELiteralSelector.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/EqHelper.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/FlatTerm.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Formula.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/FormulaTransformer.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/FormulaUnit.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/FormulaVarIterator.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Grounder.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Inference.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/InferenceStore.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/InterpretedLiteralEvaluator.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Rebalancing.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/KBO.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/KBOForEPR.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/LiteralSelector.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/LookaheadLiteralSelector.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/MainLoop.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Matcher.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/MaximalLiteralSelector.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/MLMatcher.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/MLMatcherSD.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/MLVariant.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Ordering.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Ordering_Equality.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Problem.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Renaming.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/RobSubstitution.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/HOLUnification.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/HOLMatching.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/MismatchHandler.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Signature.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/SortHelper.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/OperatorType.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/SpassLiteralSelector.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/RndLiteralSelector.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/SubformulaIterator.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Substitution.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Term.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/TermIterators.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/TermTransformer.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Theory.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Signature.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Unit.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/BottomUpEvaluation.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/BottomUpEvaluation/TermList.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/BottomUpEvaluation/TypedTermList.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/BottomUpEvaluation/PolyNf.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/BestLiteralSelector.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Clause.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/ClauseQueue.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/ColorHelper.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Connective.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/ELiteralSelector.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/EqHelper.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/FlatTerm.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Formula.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/FormulaTransformer.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/FormulaUnit.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/FormulaVarIterator.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Grounder.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Inference.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/InferenceStore.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/InterpretedLiteralEvaluator.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Rebalancing.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/KBO.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/KBOForEPR.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/LiteralComparators.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/LiteralSelector.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/LookaheadLiteralSelector.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/MainLoop.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Matcher.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/MaximalLiteralSelector.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/MLMatcher.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/MLVariant.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Ordering.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Problem.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/RCClauseStack.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Renaming.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/RobSubstitution.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/HOLUnification.hpp 
    ${CMAKE_SOURCE_DIR}/src/Kernel/HOLMatching.hpp   
    ${CMAKE_SOURCE_DIR}/src/Kernel/MismatchHandler.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Signature.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/SortHelper.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/OperatorType.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/SpassLiteralSelector.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/RndLiteralSelector.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/SubformulaIterator.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/SubstHelper.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Substitution.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Term.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/TermIterators.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/TermTransformer.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Theory.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Signature.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Unit.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/LPO.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/LPO.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Polynomial.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Polynomial.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/PolynomialNormalizer.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/PolynomialNormalizer.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/ApplicativeHelper.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/ApplicativeHelper.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/CNFOnTheFly.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/CNFOnTheFly.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/BetaEtaISE.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/BetaEtaISE.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/FlexFlexSimplify.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/FlexFlexSimplify.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ImitateProject.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ImitateProject.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ArgCong.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ArgCong.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/NegativeExt.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/NegativeExt.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/PositiveExt.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/PositiveExt.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/BoolEqToDiseq.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/BoolEqToDiseq.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/PrimitiveInstantiation.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/PrimitiveInstantiation.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ElimLeibniz.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ElimLeibniz.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Choice.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Choice.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Injectivity.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Injectivity.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/BoolSimp.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/BoolSimp.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/CasesSimp.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/CasesSimp.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Cases.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Cases.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/LambdaConversion.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/LambdaConversion.hpp
    )
source_group(kernel_source_files FILES ${VAMPIRE_KERNEL_SOURCES})

set(VAMPIRE_INDEXING_SOURCES
    ${CMAKE_SOURCE_DIR}/src/Indexing/AcyclicityIndex.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/ClauseCodeTree.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/ClauseVariantIndex.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/CodeTree.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/CodeTreeInterfaces.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/GroundingIndex.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/Index.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/IndexManager.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/InductionFormulaIndex.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/LiteralIndex.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/LiteralMiniIndex.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/LiteralSubstitutionTree.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/ResultSubstitution.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/SubstitutionTree.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/HOLSubstitutionTree.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/SubstitutionTree_FastGen.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/SubstitutionTree_FastInst.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/SubstitutionTree_Nodes.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/TermCodeTree.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/TermIndex.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/TermSharing.cpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/TermSubstitutionTree.cpp
#    ${CMAKE_SOURCE_DIR}/src/Indexing/AcyclicityIndex.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/ClauseCodeTree.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/ClauseVariantIndex.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/CodeTree.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/CodeTreeInterfaces.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/GroundingIndex.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/Index.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/IndexManager.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/InductionFormulaIndex.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/LiteralIndex.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/LiteralIndexingStructure.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/LiteralMiniIndex.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/LiteralSubstitutionTree.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/ResultSubstitution.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/SubstitutionTree.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/HOLSubstitutionTree.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/TermCodeTree.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/TermIndex.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/TermIndexingStructure.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/TermSharing.hpp
    ${CMAKE_SOURCE_DIR}/src/Indexing/TermSubstitutionTree.hpp
    ${CMAKE_SOURCE_DIR}/src/)
source_group(indexing_source_files FILES ${VAMPIRE_INDEXING_SOURCES})

set(VAMPIRE_INFERENCE_SOURCES
    ${CMAKE_SOURCE_DIR}/src/Inferences/BackwardDemodulation.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/BackwardSubsumptionDemodulation.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/BackwardSubsumptionResolution.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/BinaryResolution.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Condensation.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/DistinctEqualitySimplifier.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/EqualityFactoring.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/EqualityResolution.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ExtensionalityResolution.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Factoring.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/FastCondensation.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/FOOLParamodulation.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ForwardDemodulation.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ForwardLiteralRewriting.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ForwardSubsumptionAndResolution.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ForwardSubsumptionDemodulation.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/GlobalSubsumption.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/HyperSuperposition.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/InnerRewriting.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/EquationalTautologyRemoval.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Induction.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/InductionHelper.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/InferenceEngine.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Instantiation.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/InterpretedEvaluation.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/PushUnaryMinus.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Cancellation.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ArithmeticSubtermGeneralization.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/NumTraits.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/GaussianVariableElimination.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Rebalancing.cpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Rebalancing/Inverters.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/SLQueryBackwardSubsumption.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Superposition.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/TautologyDeletionISE.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/TermAlgebraReasoning.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/URResolution.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/BackwardDemodulation.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/BackwardSubsumptionResolution.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/BinaryResolution.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Condensation.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/DistinctEqualitySimplifier.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/EqualityFactoring.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/EqualityResolution.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ExtensionalityResolution.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Factoring.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/FastCondensation.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/FOOLParamodulation.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ForwardDemodulation.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ForwardLiteralRewriting.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ForwardSubsumptionAndResolution.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/GlobalSubsumption.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/HyperSuperposition.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/InnerRewriting.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/EquationalTautologyRemoval.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/InductionHelper.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/InferenceEngine.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Instantiation.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/InterpretedEvaluation.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/PushUnaryMinus.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Cancellation.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ArithmeticSubtermGeneralization.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/NumTraits.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/GaussianVariableElimination.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Rebalancing.hpp
    ${CMAKE_SOURCE_DIR}/src/Kernel/Rebalancing/Inverters.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/SLQueryBackwardSubsumption.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/SubsumptionDemodulationHelper.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Superposition.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/TautologyDeletionISE.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/TermAlgebraReasoning.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/URResolution.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/TheoryInstAndSimp.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/TheoryInstAndSimp.cpp  # this is theory instantiation
    ${CMAKE_SOURCE_DIR}/src/Inferences/ArithmeticSubtermGeneralization.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/ArithmeticSubtermGeneralization.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/PolynomialEvaluation.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/PolynomialEvaluation.cpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Cancellation.hpp
    ${CMAKE_SOURCE_DIR}/src/Inferences/Cancellation.cpp
    )
source_group(inference_source_files FILES ${VAMPIRE_INFERENCE_SOURCES})

set(VAMPIRE_INSTANCEGENERATION_SOURCES
    ${CMAKE_SOURCE_DIR}/src/InstGen/IGAlgorithm.cpp
    ${CMAKE_SOURCE_DIR}/src/InstGen/ModelPrinter.cpp
    ${CMAKE_SOURCE_DIR}/src/InstGen/IGAlgorithm.hpp
    ${CMAKE_SOURCE_DIR}/src/InstGen/ModelPrinter.hpp
    ${CMAKE_SOURCE_DIR}/src/)
source_group(instancegeneration_source_files FILES ${VAMPIRE_INSTANCEGENERATION_SOURCES})

set(VAMPIRE_SAT_SOURCES
    ${CMAKE_SOURCE_DIR}/src/SAT/BufferedSolver.cpp
    ${CMAKE_SOURCE_DIR}/src/SAT/FallbackSolverWrapper.cpp
    ${CMAKE_SOURCE_DIR}/src/SAT/MinimizingSolver.cpp
    ${CMAKE_SOURCE_DIR}/src/SAT/SAT2FO.cpp
    ${CMAKE_SOURCE_DIR}/src/SAT/SATClause.cpp
    ${CMAKE_SOURCE_DIR}/src/SAT/SATInference.cpp
    ${CMAKE_SOURCE_DIR}/src/SAT/SATLiteral.cpp
    ${CMAKE_SOURCE_DIR}/src/SAT/Z3Interfacing.cpp

    ${CMAKE_SOURCE_DIR}/src/SAT/BufferedSolver.hpp
    ${CMAKE_SOURCE_DIR}/src/SAT/FallbackSolverWrapper.hpp
    ${CMAKE_SOURCE_DIR}/src/SAT/MinimizingSolver.hpp
    ${CMAKE_SOURCE_DIR}/src/SAT/SAT2FO.hpp
    ${CMAKE_SOURCE_DIR}/src/SAT/SATClause.hpp
    ${CMAKE_SOURCE_DIR}/src/SAT/SATInference.hpp
    ${CMAKE_SOURCE_DIR}/src/SAT/SATLiteral.hpp
    ${CMAKE_SOURCE_DIR}/src/SAT/SATSolver.hpp
    ${CMAKE_SOURCE_DIR}/src/SAT/Z3Interfacing.hpp
    ${CMAKE_SOURCE_DIR}/src/)
source_group(sat_source_files FILES ${VAMPIRE_SAT_SOURCES})

set(VAMPIRE_DECISION_PROCEDURES_SOURCES
    ${CMAKE_SOURCE_DIR}/src/DP/ShortConflictMetaDP.cpp
    ${CMAKE_SOURCE_DIR}/src/DP/SimpleCongruenceClosure.cpp
    ${CMAKE_SOURCE_DIR}/src/DP/DecisionProcedure.hpp
    ${CMAKE_SOURCE_DIR}/src/DP/ShortConflictMetaDP.hpp
    ${CMAKE_SOURCE_DIR}/src/DP/SimpleCongruenceClosure.hpp
    ${CMAKE_SOURCE_DIR}/src/)
source_group(decision_procedures_source_files FILES ${VAMPIRE_DECISION_PROCEDURES_SOURCES})

set(VAMPIRE_SATURATION_SOURCES
    ${CMAKE_SOURCE_DIR}/src/Saturation/AWPassiveClauseContainer.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/ManCSPassiveClauseContainer.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/ClauseContainer.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/ConsequenceFinder.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/Discount.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/ExtensionalityClauseContainer.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/LabelFinder.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/LRS.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/Otter.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/ProvingHelper.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/SaturationAlgorithm.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/Splitter.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/SymElOutput.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/PredicateSplitPassiveClauseContainer.cpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/AWPassiveClauseContainer.hpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/ClauseContainer.hpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/ConsequenceFinder.hpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/Discount.hpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/ExtensionalityClauseContainer.hpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/LabelFinder.hpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/LRS.hpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/Otter.hpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/ProvingHelper.hpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/SaturationAlgorithm.hpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/Splitter.hpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/SymElOutput.hpp
    ${CMAKE_SOURCE_DIR}/src/Saturation/PredicateSplitPassiveClauseContainer.hpp
    )
source_group(saturation_source_files FILES ${VAMPIRE_SATURATION_SOURCES})

set(VAMPIRE_SHELL_SOURCES
    ${CMAKE_SOURCE_DIR}/src/Shell/AnswerExtractor.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/CommandLine.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/CNF.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/NewCNF.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/NameReuse.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/DistinctProcessor.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/DistinctGroupExpansion.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/EqResWithDeletion.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/EqualityProxy.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/EqualityProxyMono.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Flattening.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/FunctionDefinition.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/GeneralSplitting.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/GoalGuessing.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/InequalitySplitting.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/InterpolantMinimizer.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Interpolants.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/InterpretedNormalizer.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/LaTeX.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Lexer.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/LispLexer.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/LispParser.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Naming.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/NNF.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Normalisation.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Shuffling.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Shuffling.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Options.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/PredicateDefinition.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Preprocess.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Property.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Rectify.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Skolem.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/SimplifyFalseTrue.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/SineUtils.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/FOOLElimination.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Statistics.cpp
    ${CMAKE_SOURCE_DIR}/src/Debug/TimeProfiling.hpp
    ${CMAKE_SOURCE_DIR}/src/Debug/TimeProfiling.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/SymbolDefinitionInlining.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/SymbolOccurrenceReplacement.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/SymCounter.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/TermAlgebra.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/TheoryAxioms.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/TheoryFinder.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/TheoryFlattening.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/BlockedClauseElimination.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Token.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/TPTPPrinter.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/TweeGoalTransformation.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/UIHelper.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/VarManager.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Lexer.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Preprocess.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/AnswerExtractor.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/CommandLine.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/CNF.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/NewCNF.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/NameReuse.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/DistinctProcessor.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/DistinctGroupExpansion.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/EqResWithDeletion.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/EqualityProxy.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/EqualityProxyMono.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Flattening.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/FunctionDefinition.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/GeneralSplitting.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/InequalitySplitting.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/InterpolantMinimizer.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Interpolants.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/InterpretedNormalizer.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/LaTeX.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Lexer.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/LispLexer.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/LispParser.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Naming.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/NNF.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Normalisation.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Options.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/PredicateDefinition.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Preprocess.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Property.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Rectify.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Skolem.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/SimplifyFalseTrue.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/SineUtils.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/SMTLIBLogic.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/FOOLElimination.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Statistics.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/SymbolDefinitionInlining.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/SymbolOccurrenceReplacement.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/SymCounter.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/TermAlgebra.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/TheoryAxioms.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/TheoryFinder.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/TheoryFlattening.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/BlockedClauseElimination.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Token.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/TPTPPrinter.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/TweeGoalTransformation.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/UIHelper.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/VarManager.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Lexer.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/Preprocess.hpp
    ${CMAKE_SOURCE_DIR}/src/Shell/SubexpressionIterator.cpp
    ${CMAKE_SOURCE_DIR}/src/Shell/SubexpressionIterator.hpp
    )
source_group(shell_source_files FILES ${VAMPIRE_SHELL_SOURCES})

set(VAMPIRE_PARSE_SOURCES
    ${CMAKE_SOURCE_DIR}/src/Parse/SMTLIB2.cpp
    ${CMAKE_SOURCE_DIR}/src/Parse/TPTP.cpp
    ${CMAKE_SOURCE_DIR}/src/Parse/SMTLIB2.hpp
    ${CMAKE_SOURCE_DIR}/src/Parse/TPTP.hpp
    )
source_group(parse_source_files FILES ${VAMPIRE_PARSE_SOURCES})

set(VAMPIRE_FINITEMODELBUILDING_SOURCES
    ${CMAKE_SOURCE_DIR}/src/FMB/ClauseFlattening.cpp
    ${CMAKE_SOURCE_DIR}/src/FMB/FiniteModel.cpp
    ${CMAKE_SOURCE_DIR}/src/FMB/FiniteModelBuilder.cpp
    ${CMAKE_SOURCE_DIR}/src/FMB/FiniteModelMultiSorted.cpp
    ${CMAKE_SOURCE_DIR}/src/FMB/FunctionRelationshipInference.cpp
    ${CMAKE_SOURCE_DIR}/src/FMB/Monotonicity.cpp
    ${CMAKE_SOURCE_DIR}/src/FMB/SortInference.cpp
    ${CMAKE_SOURCE_DIR}/src/FMB/ClauseFlattening.hpp
    ${CMAKE_SOURCE_DIR}/src/FMB/DefinitionIntroduction.hpp
    ${CMAKE_SOURCE_DIR}/src/FMB/FiniteModel.hpp
    ${CMAKE_SOURCE_DIR}/src/FMB/FiniteModelBuilder.hpp
    ${CMAKE_SOURCE_DIR}/src/FMB/FiniteModelMultiSorted.hpp
    ${CMAKE_SOURCE_DIR}/src/FMB/FunctionRelationshipInference.hpp
    ${CMAKE_SOURCE_DIR}/src/FMB/ModelCheck.hpp
    ${CMAKE_SOURCE_DIR}/src/FMB/Monotonicity.hpp
    ${CMAKE_SOURCE_DIR}/src/FMB/SortInference.hpp
    )
source_group(finitemodelbuilding_source_files FILES ${VAMPIRE_FINITEMODELBUILDING_SOURCES})

set(VAMPIRE_SMTCOMP_SOURCES
    ${CMAKE_SOURCE_DIR}/src/SAT/Z3MainLoop.cpp
    ${CMAKE_SOURCE_DIR}/src/SAT/Z3MainLoop.hpp
    )
source_group(smt_comp_source_files FILES ${VAMPIRE_SMTCOMP_SOURCES})

set(VAMPIRE_CASC_SOURCES
    ${CMAKE_SOURCE_DIR}/src/CASC/PortfolioMode.cpp
    ${CMAKE_SOURCE_DIR}/src/CASC/Schedules.cpp
    ${CMAKE_SOURCE_DIR}/src/CASC/CLTBMode.cpp
    ${CMAKE_SOURCE_DIR}/src/CASC/CLTBModeLearning.cpp
    ${CMAKE_SOURCE_DIR}/src/CASC/PortfolioMode.hpp
    ${CMAKE_SOURCE_DIR}/src/CASC/Schedules.hpp
    ${CMAKE_SOURCE_DIR}/src/CASC/CLTBMode.hpp
    ${CMAKE_SOURCE_DIR}/src/CASC/CLTBModeLearning.hpp
    )
source_group(casc_source_files FILES ${VAMPIRE_CASC_SOURCES})

set(VAMPIRE_TESTING_SOURCES
    ${CMAKE_SOURCE_DIR}/src/Test/UnitTesting.cpp
    ${CMAKE_SOURCE_DIR}/src/Test/UnitTesting.hpp
    ${CMAKE_SOURCE_DIR}/src/Test/SyntaxSugar.hpp
    ${CMAKE_SOURCE_DIR}/src/Test/SyntaxSugar.cpp
    ${CMAKE_SOURCE_DIR}/src/Test/TestUtils.hpp
    ${CMAKE_SOURCE_DIR}/src/Test/TestUtils.cpp
    )
source_group(testing_files FILES ${VAMPIRE_TESTING_SOURCES})

set(UNIT_TESTS
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tDHMap.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tQuotientE.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tUnificationWithAbstraction.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tHOLMatching.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tGaussianElimination.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tPushUnaryMinus.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tArithmeticSubtermGeneralization.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tInterpretedFunctions.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tRebalance.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tDisagreement.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tDynamicHeap.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tInduction.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tIntegerConstantType.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tSATSolver.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tArithCompare.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tSyntaxSugar.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tSkipList.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tBinaryHeap.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tSafeRecursion.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tKBO.cpp
#    ${CMAKE_SOURCE_DIR}/src/UnitTests/tSKIKBO.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tLPO.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tRatioKeeper.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tTwoVampires.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tOptionConstraints.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tDHMultiset.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tList.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tBottomUpEvaluation.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tCoproduct.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tEqualityResolution.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tEqualityFactoring.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tImitateProject.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tIterator.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tOption.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tStack.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tBetaReduction.cpp
    )
source_group(unit_tests FILES ${UNIT_TESTS})

set(UNIT_TESTS_Z3
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tTheoryInstAndSimp.cpp
    ${CMAKE_SOURCE_DIR}/src/UnitTests/tZ3Interfacing.cpp
    )
source_group(unit_tests_z3 FILES ${UNIT_TESTS_Z3})

AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src/Lib SOURCES_LIB)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src/Lib/Sys SOURCES_LIB_SYS)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src/Sat SOURCES_SAT)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src/Saturation SOURCES_SATURATION)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src/Parse SOURCES_PARSE)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src/Shell SOURCES_SHELL)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src/Kernel SOURCES_KERNEL)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src/Indexing SOURCES_INDEXING)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src/Inferences SOURCES_INFERENCES)
AUX_SOURCE_DIRECTORY(${CMAKE_SOURCE_DIR}/src/FMB SOURCES_FMB)
add_compile_definitions(VTIME_PROFILING=1)
add_compile_definitions(VHOL=1)
add_compile_definitions(VDEBUG=0)
configure_file(src/version.cpp.in version.cpp)
add_executable(vampire
  src/vampire_node.cpp
  ${VAMPIRE_DEBUG_SOURCES}
    ${VAMPIRE_LIB_SOURCES}
    ${VAMPIRE_LIB_SYS_SOURCES}
    ${VAMPIRE_KERNEL_SOURCES}
    ${VAMPIRE_INDEXING_SOURCES}
    ${VAMPIRE_INFERENCE_SOURCES}
    ${VAMPIRE_INSTANCEGENERATION_SOURCES}
    ${VAMPIRE_SAT_SOURCES}
    ${VAMPIRE_DECISION_PROCEDURES_SOURCES}
    ${VAMPIRE_SATURATION_SOURCES}
    ${VAMPIRE_SHELL_SOURCES}
    ${VAMPIRE_PARSE_SOURCES}
    ${VAMPIRE_FINITEMODELBUILDING_SOURCES}
    ${VAMPIRE_SMTCOMP_SOURCES}
    ${VAMPIRE_MINISAT_SOURCES}
    ${VAMPIRE_CASC_SOURCES}
    src/Forwards.hpp
    "${CMAKE_CURRENT_BINARY_DIR}/version.cpp")
    #     ${SOURCES_KERNEL}
    #${SOURCES_LIB}
    #${SOURCES_FMB}
    #${SOURCES_SAT}
    #${SOURCES_SATURATION}
    #${SOURCES_PARSE}
    #${SOURCES_INFERENCES}
    #${SOURCES_LIB_SYS}
    #   ${SOURCES_INDEXING}
    #${SOURCES_SHELL})
 #target_link_directories(vampire
 # PUBLIC
 # $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src/Lib>
 #   $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vampire rclcpp std_msgs)
set(CMAKE_BUILD_TYPE )
install(TARGETS
  vampire
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
