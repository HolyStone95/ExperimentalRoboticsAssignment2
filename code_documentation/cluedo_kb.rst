
Code docs -- *cluedo_kb*
==============================

How to store a hint 
--------------------

This node reads automatically the hint (mesage ``erl2/ErlOracle``) by subscribing to the topic ``/oracle_hint`` provided by the Oracle node (namely *simulation.cpp*). Here's the message prototype:

.. code-block::
	
	int32 ID
	string key
	string value 



How to get a valid hint 
------------------------

For getting a valid hint, a request to the service server ``/get_id`` is done, so that it can receive as response, a message of type ``erl_assignment_2_msgs/GetId``. Here's the Service prototype

.. code-block::
	
	## service file 'GetId.srv'
	# empty request

	---

	# if there are yet consistent hypothese
	bool consistent_found

	# the ID of the selected consisten hp (chech consistent_found before)
	int32 consistent_id



How to discard a hint ID from the system 
-----------------------------------------

In order to discard a specific ID from the **ontology**, the service ``/mark_wrong_id`` is provided.

Here below you can find a prototype of it:

.. code-block::
	
	## service file 'MarkWrongId.srv'
	
	# ID to be discarded
	int32 ID
	
	---
	
	# empty response 
	



Code Reference
---------------

.. automodule:: cluedo_kb
	:members:
	:noindex:
