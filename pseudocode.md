Warehouse pseudocode algorithm:

Inputs:
- Warehouse graph (undirected weighted), nodes include dispatch "D", supplier "V" (optional), and shelf nodes
- Precomputed all-pairs shortest paths between nodes
- Items vector where each item has:
	a - item id
	b - shelf node
	c - current stock
	d - weight per unit
	e - item type: fragile, expensive, normal, perishable
	f - handle_time (extra minutes per unit)
	g - max_wait (perishable only)
	h - price
	i - outofstock boolean
- Orders vector where each order has:
	a - order id
	b - lines: list of (item_id, quantity)
	c - release time
	d - deadline
	e - priority: URGENT / VIP / NORMAL
	f - pack_time
- Pickers (workers) vector where each worker has:
	a - worker id
	b - shift_start, shift_end
	c - capacity (max kg per batch)
	d - overtime_limit
- Optional restock info per item:
	a - reserve
	b - prep_time, delivery_time
	c - max_weight per delivery
	d - delay_risk, delay_buffer

Helper functions:
- getItemIndexById(item_id)
- computeOrderWeight(order)
- computeOrderHandleTime(order) = sum over lines of qty * (1 + item.handle_time)
- computeOrderShelfSet(order)
- nearestNeighborRoute(shelves, start=D) using shortest paths (returns route and travel time)
- estimateBatchTime(route_travel_time, handle_time, pack_time)
- perishableOk(order, pick_time) checks max_wait since last restock arrival for each perishable item

1. Initialize restock tracking
- restocks vector holds (item_id, qty, creation_time, arrival_time, shelf, status)
- lastRestockArrival[item_id] = time 0 for initial stock, updated on each arrival

2. Restock scheduling based on demand
- For each item, compute total demanded units across all orders.
- If demand > initial stock, schedule restock deliveries:
	* required = demand - initial stock
	* while required > 0:
		- qty = min(required, max_units_by_weight(max_weight, item.weight))
		- if qty exceeds reserve, report INFEASIBLE: insufficient reserve
		- creation_time = now (planning time 0)
		- arrival_time = creation_time + prep_time + delivery_time
		- if delay_risk is high, arrival_time += delay_buffer
		- push restock request
		- required -= qty, reserve -= qty

3. Order scoring (weighted linear function)
- First, compute raw features for each order:
	priorityWeight = 3 for URGENT, 2 for VIP, 1 for NORMAL
	slack = minutes(deadline - release)
	stockUrgency = min over items in order of (1 - current_stock / max(1, total_demand_for_item))
	expectedHandle = computeOrderHandleTime(order)
	orderPrice = sum of item prices in the order
- Then normalize each feature across all orders:
	For each feature f in {priorityWeight, slack, stockUrgency, expectedHandle, orderPrice}:
		f_norm = (f - f_min) / max(1, f_max - f_min)
- For each order, compute a score:
	score = w1 * priorityWeight_norm + w2 * orderPrice_norm + w3 * stockUrgency_norm - w4 * expectedHandle_norm - w5 * slack_norm
- Sort orders by:
	1) higher score
	2) earlier deadline (tie-break only)

The weights are set heuristically: w1 = 0.35, w2 = 0.20, w3 = 0.25, w4 = 0.1, and w5 = 0.1

4. Build nextDayPlan (global greedy)
- Initialize each picker with available_time = shift_start
- Maintain a list of active pickers with their current availability
- skippedOrders list for orders waiting on stock or perishable constraint

- For each order in the sorted order list:
	* choose the picker with the earliest available_time that can still work
	  within shift_end + overtime_limit
	* start_time = max(picker.available_time, order.release)
	* if any required item is out of stock at start_time:
		- schedule restock if needed (see ensureRestock)
		- move order to skippedOrders
		- continue
	* if not perishableOk(order, start_time):
		- move order to skippedOrders
		- continue
	* attempt to add the order to the picker's current batch:
		- check capacity with current batch weight
		- if it does not fit, finalize current batch and start a new one
	* when a batch is finalized or a new batch is started:
		- compute shelves = union of shelves in batch
		- route = nearestNeighborRoute(shelves, start=D)
		- travel_time from route
		- finish_time = start_time + travel_time + handle_time + pack_time
		- dispatch_time = finish_time
		- if finish_time > shift_end + overtime_limit, mark INFEASIBLE
		- if dispatch_time > order deadline, mark INFEASIBLE
		- update stock for each item in batch at pick_time
		- picker.available_time = finish_time
		- call updateRestocks(picker.available_time)

- After processing all orders, try to assign skippedOrders in the same greedy way
	(as soon as their items are restocked and constraints are satisfied)

- ensureRestock(item_id, needed_qty, need_time, current_time):
	* lead_time = prep_time + delivery_time (+ delay_buffer if risk is high)
	* creation_time = max(current_time, need_time - lead_time)
	* arrival_time = creation_time + lead_time
	* if arrival_time > need_time, the order waits until arrival_time
	* respect reserve and max_weight per delivery

- updateRestocks(current_time):
	* for each restock with arrival_time <= current_time and status pending:
		- add qty to item stock
		- set outofstock false if stock > 0
		- update lastRestockArrival[item_id] = arrival_time

6. Re-check skipped orders when stock arrives
- After each restock update, try to assign skipped orders that now have stock
- Use the same feasibility checks as above (capacity, deadlines, max_wait)

Output:
- If feasible, output nextDayPlan with:
	* restock requests (item, qty, creation_time, arrival_time)
	* for each picker: batches, route, start/finish/dispatch times, total weight
- If infeasible, output INFEASIBLE with a specific reason

