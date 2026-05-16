/*
 * Warehouse Operations Planning — Thread B (AI-Assisted)
 * CSCE 2202 – Analysis and Design of Algorithms
 *
 * Algorithm: Priority-Driven Greedy with Constraint Propagation
 *
 * Build:  g++ -o ai_solution.cpp
 * Run:    ./ai_solution <case_number>   (1 | 2 | 3 | 4)
 *
 * Human edits documented in Thread B report (see comments marked "Human edit:").
 */

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <string>
#include <algorithm>
#include <iomanip>
#include <random>
#include <chrono>
using namespace std;

// Constants
static const double INF   = 1e18;
static const int    SEED  = 42;

// Data Types

enum class Priority { NORMAL = 0, URGENT = 1, VIP = 2 };
enum class ItemType  { NORMAL, FRAGILE, EXPENSIVE, PERISHABLE };

struct Item {
    string id;
    string shelf;       // shelf node name
    int    stock;       // current available on shelf
    double weight;      // kg per unit
    ItemType itype;
    double max_wait     = 1e18;   // perishable only: max minutes on shelf before pick
    double handle_time  = 0.0;    // extra minutes per unit for fragile/expensive
};

struct OrderLine { string item_id; int qty; };

struct Order {
    string id;
    vector<OrderLine> lines;
    double release;     // earliest time this order may be processed
    double deadline;
    Priority priority;
    double pack_time = 2.0;
};

struct Picker {
    string id;
    double shift_start;
    double shift_end;
    double capacity;        // max kg per trip
    double overtime_limit   = 30.0;
    double overtime_penalty = 5.0;   // cost per overtime minute
};

struct RestockInfo {
    string item_id;
    int    reserve;         // total units available at supplier V
    double prep_time;       // minutes to prepare at V
    double delivery_time;   // minutes from V to shelf
    double max_weight;      // kg per single restock delivery
    double delay_risk   = 0.0;
    double delay_buffer = 0.0;
};

// Planned Output Structures

struct RestockRequest {
    int    id;
    string item_id;
    int    qty;
    string shelf;
    double creation_time;
    double arrival_time;
    double delay_buffer_used;
    double latest_needed;   // must arrive before this time
};

struct PickBatch {
    int            id;
    vector<string> order_ids;
    string         picker_id;
    double         start_time;
    double         finish_time;
    double         dispatch_time;
    vector<string> shelf_sequence;
    double         total_weight;
};

struct Plan {
    vector<PickBatch>     pick_batches;
    vector<RestockRequest> restock_requests;
    bool   feasible = true;
    string infeasible_reason;
};

// Warehouse Graph
// Shortest-path precomputation via Floyd-Warshall  O(V^3) for all pairs travel times.

class Graph {
public:
    vector<string>                    nodes;
    map<string,int>                   idx;
    vector<vector<double>>            dist;

    Graph() = default;

    void add_nodes(const vector<string>& ns) {
        nodes = ns;
        int n = (int)ns.size();
        idx.clear();
        for (int i = 0; i < n; ++i) idx[ns[i]] = i;
        dist.assign(n, vector<double>(n, INF));
        for (int i = 0; i < n; ++i) dist[i][i] = 0;
    }

    void add_edge(const string& u, const string& v, double t) {
        dist[idx[u]][idx[v]] = t;
        dist[idx[v]][idx[u]] = t;   // undirected
    }

    void floyd_warshall() {
        int n = (int)nodes.size();
        for (int k = 0; k < n; ++k)
            for (int i = 0; i < n; ++i)
                for (int j = 0; j < n; ++j)
                    if (dist[i][k] < INF && dist[k][j] < INF)
                        dist[i][j] = min(dist[i][j], dist[i][k] + dist[k][j]);
    }

    double travel(const string& u, const string& v) const {
        auto iu = idx.find(u), iv = idx.find(v);
        if (iu == idx.end() || iv == idx.end()) return INF;
        return dist[iu->second][iv->second];
    }

    // Nearest-neighbour TSP from 'start' visiting all 'stops'.
    // Returns ordered visit sequence (not including return to dispatch).
    // O(stops^2), which is acceptable for batch sizes in practice.
    vector<string> tsp_greedy(const string& start,
                               vector<string> stops) const {
        vector<string> route;
        string cur = start;
        while (!stops.empty()) {
            auto best = min_element(stops.begin(), stops.end(),
                [&](const string& a, const string& b){
                    return travel(cur,a) < travel(cur,b);
                });
            cur = *best;
            route.push_back(cur);
            stops.erase(best);
        }
        return route;
    }
};

// Planner
/*
 * High-level algorithm (AI-suggested, student-verified):
 *
 * Phase 0 – Floyd-Warshall on warehouse graph.
 * Phase 1 – Stock demand analysis: for each item, compute total demand.
 *            Detect items needing restock.
 * Phase 2 – Restock scheduling (greedy, earliest-first):
 *            Compute how many restock deliveries are needed per item.
 *            Set creation_time = 0 (start of day) and propagate arrival_time.
 *            Apply delay buffer for risky items.
 * Phase 3 – Order prioritisation:
 *            Sort orders by (priority DESC, slack ASC), where
 *            slack = deadline − (release + min_possible_pick_time).
 *            VIP > URGENT > NORMAL. Ties broken by earliest deadline.
 * Phase 4 – Batch packing (greedy, capacity-aware):
 *            For each picker (sorted by shift_start), greedily fill batches
 *            respecting weight capacity.  Each batch gets orders whose
 *            total weight ≤ picker capacity.
 * Phase 5 – Route each batch with nearest-neighbour TSP.
 *            Compute pick timeline per batch including service + handle times.
 * Phase 6 – Feasibility validation:
 *            (a) Stock timeline: for every pick event, running stock ≥ 0.
 *            (b) Restock arrivals precede dependent pick events.
 *            (c) Deadlines satisfied (or report lateness).
 *            (d) Picker shift + overtime limits respected.
 * Phase 7 – Output plan or INFEASIBLE with reason.
 *
 * Complexity:
 *   Floyd-Warshall:  O(V^3)
 *   Sort orders:     O(M log M)
 *   Batch packing:   O(M * P)  where P = number of pickers
 *   Route per batch: O(S^2)    where S = shelves per batch
 *   Validation:      O(M * S)
 *   Overall:         O(V^3 + M*P + B*S^2)  where B = number of batches
 */

class WarehousePlanner {
public:
    Graph                        graph;
    map<string, Item>            items;
    vector<Order>                orders;
    vector<Picker>               pickers;
    map<string, RestockInfo>     restock_info;  // keyed by item_id

    Plan solve() {
        Plan plan;

        // == Phase 0
        graph.floyd_warshall();

        // == Phase 1: demand analysis
        map<string,int> total_demand;
        for (auto& o : orders)
            for (auto& l : o.lines)
                total_demand[l.item_id] += l.qty;

        // working stock copy (mutable throughout)
        map<string,int> shelf_stock;
        for (auto& [id, it] : items) shelf_stock[id] = it.stock;

        // == Phase 2: restock scheduling
        int rr_id = 0;
        map<string,int> reserve_used;
        // Track scheduled restock arrival per item (cumulative)
        map<string,double> last_restock_arrival;

        for (auto& [item_id, ri] : restock_info) {
            int demand   = total_demand.count(item_id) ? total_demand[item_id] : 0;
            int shortfall = demand - shelf_stock[item_id];
            if (shortfall <= 0) continue;

            if (ri.reserve < shortfall) {
                plan.feasible = false;
                plan.infeasible_reason = "Insufficient reserve stock for item " + item_id
                    + ": need " + to_string(shortfall)
                    + ", have " + to_string(ri.reserve) + ".";
                return plan;
            }

            // Human edit: Split into multiple deliveries if shortfall × weight > max_weight.
            double unit_w = items[item_id].weight;
            int    units_per_delivery = max(1, (int)floor(ri.max_weight / unit_w));
            int    remaining = shortfall;
            double creation_t = 0.0;   // schedule at start of day

            while (remaining > 0) {
                int deliver_qty = min(remaining, units_per_delivery);
                double buf = (ri.delay_risk > 0.3) ? ri.delay_buffer : 0.0;
                double arrival = creation_t + ri.prep_time + ri.delivery_time + buf;

                RestockRequest rr;
                rr.id               = rr_id++;
                rr.item_id          = item_id;
                rr.qty              = deliver_qty;
                rr.shelf            = items[item_id].shelf;
                rr.creation_time    = creation_t;
                rr.arrival_time     = arrival;
                rr.delay_buffer_used= buf;
                rr.latest_needed    = INF;   // filled in Phase 6
                plan.restock_requests.push_back(rr);

                // Don't add to shelf_stock here — validation Phase 6 tracks timeline.
                reserve_used[item_id] += deliver_qty;
                remaining -= deliver_qty;
                last_restock_arrival[item_id] = arrival;
                creation_t += 1.0;  // stagger consecutive requests by 1 min
            }
        }

        // == Phase 3: order prioritisation
        // Human edit: AI used only deadline for sorting. We added priority tiers and tie-breaking by deadline.
        auto priority_val = [](Priority p) -> int {
            return (p == Priority::VIP) ? 2 : (p == Priority::URGENT) ? 1 : 0;
        };
        vector<Order*> sorted_orders;
        for (auto& o : orders) sorted_orders.push_back(&o);
        sort(sorted_orders.begin(), sorted_orders.end(),
            [&](Order* a, Order* b){
                int pa = priority_val(a->priority), pb = priority_val(b->priority);
                if (pa != pb) return pa > pb;
                return a->deadline < b->deadline;
            });

        // == Phase 4 & 5: batch packing + routing
        // Sort pickers by shift start
        vector<Picker*> sorted_pickers;
        for (auto& p : pickers) sorted_pickers.push_back(&p);
        sort(sorted_pickers.begin(), sorted_pickers.end(),
            [](Picker* a, Picker* b){ return a->shift_start < b->shift_start; });

        // Picker availability times
        map<string, double> picker_free;   // when picker is next free
        for (auto& p : pickers) picker_free[p.id] = p.shift_start;

        int batch_id = 0;
        vector<bool> assigned(sorted_orders.size(), false);

        // Round-robin pass: each picker takes ONE batch per round until all assigned
        bool progress = true;
        while (progress) {
            progress = false;
            for (auto* pk : sorted_pickers) {
            double t = picker_free[pk->id];
            if (t > pk->shift_end + pk->overtime_limit) continue;

            // Build ONE batch for this picker in this round
            {
                PickBatch batch;
                batch.id        = batch_id++;
                batch.picker_id = pk->id;
                batch.start_time= t;

                double batch_weight = 0;
                set<string> shelves_needed;

                for (int i = 0; i < (int)sorted_orders.size(); ++i) {
                    if (assigned[i]) continue;
                    Order* o = sorted_orders[i];
                    if (o->release > t) continue;   // not yet released

                    // Compute weight of this order
                    double ord_w = 0;
                    for (auto& l : o->lines)
                        ord_w += l.qty * items[l.item_id].weight;

                    // Don't overfill: stop once batch has >= 8 shelves or weight near cap
                    if (batch_weight + ord_w > pk->capacity) continue;
                    if ((int)shelves_needed.size() >= 8) continue;

                    // Check that required restocks for this order's items will have arrived
                    // before this batch starts. If not, skip order for now.
                    bool restock_ready = true;
                    for (auto& l : o->lines) {
                        if (!restock_info.count(l.item_id)) continue;
                        // Find the latest restock needed for this item
                        for (auto& rr : plan.restock_requests) {
                            if (rr.item_id == l.item_id && rr.arrival_time > t) {
                                restock_ready = false;
                                break;
                            }
                        }
                        if (!restock_ready) break;
                    }
                    if (!restock_ready) continue;

                    // Assign
                    batch.order_ids.push_back(o->id);
                    batch_weight += ord_w;
                    for (auto& l : o->lines)
                        shelves_needed.insert(items[l.item_id].shelf);
                    assigned[i] = true;
                }

                if (batch.order_ids.empty()) { --batch_id; continue; }
                progress = true;

                batch.total_weight = batch_weight;

                // Route: dispatch → shelves (TSP NN) → dispatch
                vector<string> stops(shelves_needed.begin(), shelves_needed.end());
                batch.shelf_sequence = graph.tsp_greedy("D", stops);

                // Compute finish time
                double cur_time = t;
                string cur_loc  = "D";
                double pick_service = 0;

                // Accumulate service time per shelf (all items at that shelf)
                map<string,double> shelf_service;
                for (auto& oid : batch.order_ids) {
                    Order* o = nullptr;
                    for (auto& ord : orders) if (ord.id == oid) { o = &ord; break; }
                    for (auto& l : o->lines) {
                        const Item& it = items[l.item_id];
                        double svc = l.qty * (1.0 + it.handle_time); // 1 min per unit baseline
                        shelf_service[it.shelf] += svc;
                    }
                }

                for (auto& shelf : batch.shelf_sequence) {
                    cur_time += graph.travel(cur_loc, shelf);
                    cur_time += shelf_service[shelf];
                    cur_loc   = shelf;
                }
                // Return to dispatch
                cur_time += graph.travel(cur_loc, "D");

                // Packing/verification time (max over all orders in batch)
                double max_pack = 0;
                for (auto& oid : batch.order_ids) {
                    for (auto& ord : orders)
                        if (ord.id == oid) { max_pack = max(max_pack, ord.pack_time); break; }
                }
                cur_time += max_pack;

                batch.finish_time   = cur_time;
                batch.dispatch_time = cur_time;

                plan.pick_batches.push_back(batch);
                picker_free[pk->id] = cur_time;
            } // end one batch
            } // end picker loop
        } // end while(progress)

        // Check unassigned orders
        for (int i = 0; i < (int)sorted_orders.size(); ++i) {
            if (!assigned[i]) {
                plan.feasible = false;
                plan.infeasible_reason = "Order " + sorted_orders[i]->id
                    + " could not be assigned (capacity or shift limits).";
                return plan;
            }
        }

        // == Phase 6: feasibility validation
        // (a) Deadline check
        map<string, double> order_dispatch;
        for (auto& b : plan.pick_batches)
            for (auto& oid : b.order_ids)
                order_dispatch[oid] = b.dispatch_time;

        for (auto& o : orders) {
            if (order_dispatch.count(o.id) && order_dispatch[o.id] > o.deadline) {
                plan.feasible = false;
                plan.infeasible_reason = "Order " + o.id
                    + " misses deadline (dispatch=" + to_string(order_dispatch[o.id])
                    + ", deadline=" + to_string(o.deadline) + ").";
                return plan;
            }
        }

        // (b) Stock timeline validation per batch
        // Running stock: start with initial stock; add restock arrivals at correct time.
        // Human edit: AI missed perishable window check, re-did it.
        map<string,int> running_stock;
        for (auto& [id, it] : items) running_stock[id] = it.stock;

        // Incorporate restocks (apply when arrival_time ≤ batch start_time)
        // Sort batches by start_time for timeline traversal
        auto sorted_batches = plan.pick_batches;
        sort(sorted_batches.begin(), sorted_batches.end(),
            [](const PickBatch& a, const PickBatch& b){ return a.start_time < b.start_time; });

        // Per-item restock queue sorted by arrival
        map<string, vector<pair<double,int>>> restock_queue; // item_id → [(arrival, qty)]
        for (auto& rr : plan.restock_requests)
            restock_queue[rr.item_id].push_back({rr.arrival_time, rr.qty});
        for (auto& [id, q] : restock_queue)
            sort(q.begin(), q.end());

        map<string, size_t> rq_ptr;  // next undelivered restock per item

        for (auto& b : sorted_batches) {
            // Apply all restocks arriving before this batch starts
            for (auto& [item_id, q] : restock_queue) {
                size_t& ptr = rq_ptr[item_id];
                while (ptr < q.size() && q[ptr].first <= b.start_time) {
                    running_stock[item_id] += q[ptr].second;
                    ++ptr;
                }
            }

            // Simulate picks along shelf sequence
            // Determine items picked at each shelf in this batch
            map<string, map<string,int>> shelf_picks; // shelf → item_id → qty
            for (auto& oid : b.order_ids) {
                Order* o = nullptr;
                for (auto& ord : orders) if (ord.id == oid) { o = &ord; break; }
                for (auto& l : o->lines)
                    shelf_picks[items[l.item_id].shelf][l.item_id] += l.qty;
            }

            double pick_time = b.start_time;
            string cur = "D";
            for (auto& shelf : b.shelf_sequence) {
                pick_time += graph.travel(cur, shelf);
                cur = shelf;

                // Apply restocks arriving up to pick_time at this shelf
                for (auto& [item_id, q] : restock_queue) {
                    if (items[item_id].shelf != shelf) continue;
                    size_t& ptr = rq_ptr[item_id];
                    while (ptr < q.size() && q[ptr].first <= pick_time) {
                        running_stock[item_id] += q[ptr].second;
                        ++ptr;
                    }
                }

                // Deduct picks
                if (shelf_picks.count(shelf)) {
                    for (auto& [item_id, qty] : shelf_picks[shelf]) {
                        running_stock[item_id] -= qty;
                        if (running_stock[item_id] < 0) {
                            plan.feasible = false;
                            plan.infeasible_reason = "Stock violation for item " + item_id
                                + " at time " + to_string(pick_time)
                                + " (restock arrives too late or insufficient).";
                            return plan;
                        }
                        // For the human edit request: Perishable check
                        const Item& it = items[item_id];
                        if (it.itype == ItemType::PERISHABLE) {
                            // Find when restock arrived (last arrival before pick_time)
                            double restock_arrival = 0;
                            for (auto& rr : plan.restock_requests)
                                if (rr.item_id == item_id && rr.arrival_time <= pick_time)
                                    restock_arrival = max(restock_arrival, rr.arrival_time);
                            if (pick_time - restock_arrival > it.max_wait) {
                                plan.feasible = false;
                                plan.infeasible_reason = "Perishable item " + item_id
                                    + " waited too long on shelf (waited "
                                    + to_string(pick_time - restock_arrival)
                                    + " min, limit " + to_string(it.max_wait) + " min).";
                                return plan;
                            }
                        }
                        pick_time += qty * (1.0 + it.handle_time);
                    }
                }
            }
        }

        // (c) Picker overtime check
        for (auto& pk : pickers) {
            double free_t = picker_free[pk.id];
            if (free_t > pk.shift_end + pk.overtime_limit) {
                plan.feasible = false;
                plan.infeasible_reason = "Picker " + pk.id
                    + " exceeds overtime limit (finishes at " + to_string(free_t)
                    + ", max=" + to_string(pk.shift_end + pk.overtime_limit) + ").";
                return plan;
            }
        }

        return plan;
    }
};

// Output Helper

void print_plan(const Plan& plan, const WarehousePlanner& planner) {
    if (!plan.feasible) {
        cout << "\n=== INFEASIBLE ===\n";
        cout << "Reason: " << plan.infeasible_reason << "\n";
        return;
    }

    cout << "\n=== FEASIBLE PLAN ===\n\n";

    cout << "── Restock Requests ──────────────────────────────────\n";
    if (plan.restock_requests.empty()) {
        cout << "  (none required — shelf stock sufficient)\n";
    }
    for (auto& rr : plan.restock_requests) {
        cout << "  RR-" << rr.id
             << "  item=" << rr.item_id
             << "  qty=" << rr.qty
             << "  shelf=" << rr.shelf
             << "  created@" << fixed << setprecision(1) << rr.creation_time
             << "  arrives@" << rr.arrival_time;
        if (rr.delay_buffer_used > 0)
            cout << "  [buffer=" << rr.delay_buffer_used << "min]";
        cout << "\n";
    }

    cout << "\n── Pick Batches ──────────────────────────────────────\n";
    for (auto& b : plan.pick_batches) {
        cout << "  Batch-" << b.id
             << "  picker=" << b.picker_id
             << "  weight=" << fixed << setprecision(1) << b.total_weight << "kg"
             << "  start=" << b.start_time
             << "  dispatch=" << b.dispatch_time << "\n";
        cout << "    Orders: ";
        for (auto& oid : b.order_ids) cout << oid << " ";
        cout << "\n";
        cout << "    Route:  D";
        for (auto& s : b.shelf_sequence) cout << " → " << s;
        cout << " → D\n";
    }
    cout << "\n";
}

// Test Case Generators

WarehousePlanner build_case1() {
    // Case 1: 4 pickers, 40 shelves, 120 orders — normal day, all stock sufficient.
    mt19937 rng(SEED);
    WarehousePlanner wp;

    // Nodes: D + 40 shelves
    vector<string> nodes = {"D"};
    for (int i = 1; i <= 40; ++i) nodes.push_back("S" + to_string(i));
    wp.graph.add_nodes(nodes);

    // Random travel times D↔shelf: 2–15 min; shelf↔shelf: 1–10 min
    auto rnd = [&](double lo, double hi) -> double {
        return lo + (hi - lo) * (rng() / (double)rng.max());
    };
    for (int i = 1; i <= 40; ++i)
        wp.graph.add_edge("D", "S"+to_string(i), rnd(2,15));
    for (int i = 1; i <= 40; ++i)
        for (int j = i+1; j <= 40; ++j)
            wp.graph.add_edge("S"+to_string(i), "S"+to_string(j), rnd(1,10));
    wp.graph.floyd_warshall();

    // Items: 80 items across 40 shelves (2 per shelf)
    for (int s = 1; s <= 40; ++s) {
        for (int k = 0; k < 2; ++k) {
            string iid = "I" + to_string((s-1)*2+k+1);
            Item it;
            it.id    = iid;
            it.shelf = "S" + to_string(s);
            it.stock = 50 + (int)(rng() % 50);   // 50–99 units
            it.weight = 0.5 + rnd(0,3.0);
            it.itype  = ItemType::NORMAL;
            wp.items[iid] = it;
        }
    }

    // 4 pickers with different shift end times
    wp.pickers = {
        {"P1", 0, 480, 50},   // 8h shift, 50 kg cap
        {"P2", 0, 420, 45},
        {"P3", 30, 510, 55},
        {"P4", 60, 540, 60},
    };

    // 120 orders with 2–3 lines each; some urgent
    for (int o = 1; o <= 120; ++o) {
        Order ord;
        ord.id = "O" + to_string(o);
        ord.release  = 0;
        ord.deadline = 400 + (int)(rng() % 100);
        ord.priority = (o % 7 == 0) ? Priority::URGENT : Priority::NORMAL;
        ord.pack_time = 2.0;
        int lines = 2 + (int)(rng() % 2);
        for (int l = 0; l < lines; ++l) {
            string iid = "I" + to_string(1 + (int)(rng() % 80));
            ord.lines.push_back({iid, 1 + (int)(rng() % 3)});
        }
        wp.orders.push_back(ord);
    }

    // No restock needed (stock sufficient)
    return wp;
}

WarehousePlanner build_case2() {
    // Case 2: 5 pickers, 60 shelves, 180 orders — one understocked item X.
    mt19937 rng(SEED);
    WarehousePlanner wp;

    vector<string> nodes = {"D","V"};
    for (int i = 1; i <= 60; ++i) nodes.push_back("S" + to_string(i));
    wp.graph.add_nodes(nodes);

    auto rnd = [&](double lo, double hi) -> double {
        return lo + (hi-lo)*(rng()/(double)rng.max());
    };
    wp.graph.add_edge("D","V",10);
    for (int i = 1; i <= 60; ++i) {
        wp.graph.add_edge("D","S"+to_string(i), rnd(2,15));
        wp.graph.add_edge("V","S"+to_string(i), rnd(5,20));
    }
    for (int i = 1; i <= 60; ++i)
        for (int j = i+1; j <= 60; ++j)
            wp.graph.add_edge("S"+to_string(i),"S"+to_string(j), rnd(1,10));
    wp.graph.floyd_warshall();

    // Items
    for (int s = 1; s <= 60; ++s) {
        string iid = "I" + to_string(s);
        Item it;
        it.id = iid; it.shelf = "S"+to_string(s);
        it.stock = 40 + (int)(rng()%30);
        it.weight = 0.5 + rnd(0,2); it.itype = ItemType::NORMAL;
        wp.items[iid] = it;
    }
    // Item X — heavily understocked
    wp.items["IX"] = {"IX","S10", 5, 1.5, ItemType::NORMAL};

    // 5 pickers
    wp.pickers = {
        {"P1",0,480,50}, {"P2",0,480,50}, {"P3",0,480,50},
        {"P4",0,480,50}, {"P5",60,540,55},
    };

    // 180 orders; ~30 require item X
    for (int o = 1; o <= 180; ++o) {
        Order ord;
        ord.id = "O"+to_string(o);
        ord.release = 0;
        ord.deadline = 350 + (int)(rng()%150);
        ord.priority = (o % 15 == 0) ? Priority::URGENT : Priority::NORMAL;
        ord.pack_time = 2.0;
        if (o % 6 == 0) {
            ord.lines.push_back({"IX", 1 + (int)(rng()%3)});
        } else {
            string iid = "I"+to_string(1+(int)(rng()%60));
            ord.lines.push_back({iid, 1+(int)(rng()%3)});
        }
        wp.orders.push_back(ord);
    }

    // Restock for IX
    RestockInfo ri;
    ri.item_id = "IX"; ri.reserve = 200;
    ri.prep_time = 15; ri.delivery_time = 30;   // 45 min total per spec
    ri.max_weight = 50; ri.delay_risk = 0.1; ri.delay_buffer = 5;
    wp.restock_info["IX"] = ri;

    return wp;
}

WarehousePlanner build_case3() {
    // Case 3: 6 pickers, 100 shelves, 300 orders — multiple understocked items,
    //         two with high delay risk requiring buffer.
    mt19937 rng(SEED);
    WarehousePlanner wp;

    vector<string> nodes = {"D","V"};
    for (int i = 1; i <= 100; ++i) nodes.push_back("S"+to_string(i));
    wp.graph.add_nodes(nodes);

    auto rnd = [&](double lo, double hi) -> double {
        return lo + (hi-lo)*(rng()/(double)rng.max());
    };
    wp.graph.add_edge("D","V",8);
    for (int i = 1; i <= 100; ++i) {
        wp.graph.add_edge("D","S"+to_string(i), rnd(2,18));
        wp.graph.add_edge("V","S"+to_string(i), rnd(5,25));
    }
    for (int i = 1; i <= 100; ++i)
        for (int j = i+1; j <= 100; ++j)
            wp.graph.add_edge("S"+to_string(i),"S"+to_string(j), rnd(1,12));
    wp.graph.floyd_warshall();

    for (int s = 1; s <= 100; ++s) {
        string iid = "I"+to_string(s);
        Item it;
        it.id = iid; it.shelf = "S"+to_string(s);
        it.stock = 20 + (int)(rng()%40);
        it.weight = 0.3 + rnd(0,2.5); it.itype = ItemType::NORMAL;
        wp.items[iid] = it;
    }
    // Three understocked items
    for (auto& [nm, sh, stk, w] : vector<tuple<string,string,int,double>>{
            {"IA","S5",3,2.0},{"IB","S20",2,1.5},{"IC","S55",4,1.0}}) {
        wp.items[nm] = {nm, sh, stk, w, ItemType::NORMAL};
    }

    wp.pickers = {
        {"P1",0,480,60},{"P2",0,480,55},{"P3",0,480,50},
        {"P4",30,510,60},{"P5",30,510,60},{"P6",60,540,70},
    };

    for (int o = 1; o <= 300; ++o) {
        Order ord;
        ord.id = "O"+to_string(o);
        ord.release = 0;
        ord.deadline = 300 + (int)(rng()%200);
        ord.priority = (o%20==0) ? Priority::VIP : (o%8==0) ? Priority::URGENT : Priority::NORMAL;
        ord.pack_time = 2.0;
        int choice = o % 10;
        if      (choice == 0) ord.lines.push_back({"IA",1+(int)(rng()%2)});
        else if (choice == 1) ord.lines.push_back({"IB",1+(int)(rng()%2)});
        else if (choice == 2) ord.lines.push_back({"IC",1+(int)(rng()%3)});
        else {
            string iid = "I"+to_string(1+(int)(rng()%100));
            ord.lines.push_back({iid, 1+(int)(rng()%4)});
        }
        wp.orders.push_back(ord);
    }

    // Restock infos: IA and IB have high delay risk
    wp.restock_info["IA"] = {"IA",100, 10, 20, 40, 0.7, 15};  // high risk → 15 min buffer
    wp.restock_info["IB"] = {"IB",100, 15, 25, 30, 0.8, 20};  // high risk → 20 min buffer
    wp.restock_info["IC"] = {"IC",100,  5, 15, 50, 0.1,  0};  // low risk

    return wp;
}

WarehousePlanner build_case4() {
    // Case 4: 3 pickers, 50 shelves, 100 orders — infeasible VIP deadline.
    mt19937 rng(SEED);
    WarehousePlanner wp;

    vector<string> nodes = {"D","V"};
    for (int i = 1; i <= 50; ++i) nodes.push_back("S"+to_string(i));
    wp.graph.add_nodes(nodes);

    auto rnd = [&](double lo, double hi) -> double {
        return lo + (hi-lo)*(rng()/(double)rng.max());
    };
    wp.graph.add_edge("D","V",5);
    for (int i = 1; i <= 50; ++i) {
        wp.graph.add_edge("D","S"+to_string(i), rnd(2,12));
        wp.graph.add_edge("V","S"+to_string(i), rnd(5,20));
    }
    for (int i = 1; i <= 50; ++i)
        for (int j = i+1; j <= 50; ++j)
            wp.graph.add_edge("S"+to_string(i),"S"+to_string(j), rnd(1,8));
    wp.graph.floyd_warshall();

    for (int s = 1; s <= 50; ++s) {
        string iid = "I"+to_string(s);
        Item it;
        it.id = iid; it.shelf = "S"+to_string(s);
        it.stock = 30 + (int)(rng()%20);
        it.weight = 0.5 + rnd(0,2); it.itype = ItemType::NORMAL;
        wp.items[iid] = it;
    }
    // VIP item — understocked; restock arrives at 80 min but VIP deadline is 60 min
    wp.items["IVIP"] = {"IVIP","S25", 0, 1.0, ItemType::NORMAL};

    wp.pickers = {
        {"P1",0,480,50}, {"P2",0,480,50}, {"P3",0,480,50},
    };

    for (int o = 1; o <= 100; ++o) {
        Order ord;
        ord.id = "O"+to_string(o);
        ord.release = 0;
        ord.deadline = 300 + (int)(rng()%150);
        ord.priority = Priority::NORMAL;
        ord.pack_time = 2.0;
        string iid = "I"+to_string(1+(int)(rng()%50));
        ord.lines.push_back({iid, 1+(int)(rng()%3)});
        wp.orders.push_back(ord);
    }
    // The infeasible VIP order
    Order vip;
    vip.id = "O_VIP"; vip.release = 0;
    vip.deadline = 60;   // tight: only 60 min
    vip.priority = Priority::VIP; vip.pack_time = 2.0;
    vip.lines.push_back({"IVIP", 5});
    wp.orders.push_back(vip);

    // Restock arrives at 20 (prep) + 60 (delivery) = 80 min — AFTER VIP deadline
    wp.restock_info["IVIP"] = {"IVIP", 100, 20, 60, 50, 0.0, 0};

    return wp;
}

// Main

int main(int argc, char* argv[]) {
    int case_num = 1;
    if (argc > 1) case_num = atoi(argv[1]);

    cout << "=== Warehouse Operations Planner — Thread B (AI-Assisted) ===\n";
    cout << "    Running Case " << case_num << "\n";

    WarehousePlanner wp;
    switch (case_num) {
        case 1: wp = build_case1(); break;
        case 2: wp = build_case2(); break;
        case 3: wp = build_case3(); break;
        case 4: wp = build_case4(); break;
        default:
            cerr << "Unknown case. Use 1–4.\n";
            return 1;
    }

    auto start = chrono::high_resolution_clock::now();
    Plan plan = wp.solve();
    auto end   = chrono::high_resolution_clock::now();
    double ms  = chrono::duration<double,milli>(end-start).count();

    print_plan(plan, wp);
    cout << "Runtime: " << fixed << setprecision(2) << ms << " ms\n";
    return 0;
}
