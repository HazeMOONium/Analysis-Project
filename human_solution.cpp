/*
 * Warehouse Operations Planning — Thread A (Human)
 * CSCE 2202 – Analysis and Design of Algorithms
 *
 * Algorithm: Score-based greedy with lead-time restock checks
 *
 * Build:  g++ -std=c++17 human_solution.cpp -o human_solution
 * Run:    ./human_solution <case_number>   (1 | 2 | 3 | 4 | 5)
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
    double price        = 0.0;    // per unit price
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


class WarehousePlanner {
public:
    Graph                        graph;
    map<string, Item>            items;
    vector<Order>                orders;
    vector<Picker>               pickers;
    map<string, RestockInfo>     restock_info;  // keyed by item_id

    Plan solve_baseline() {
        return solve_greedy(/*use_enhanced=*/false);
    }

    Plan solve_enhanced() {
        return solve_greedy(/*use_enhanced=*/true);
    }

private:
    struct FeatureRange {
        double min_v = INF;
        double max_v = -INF;
    };

    double order_weight(const Order& o) const {
        double w = 0.0;
        for (auto& l : o.lines) w += l.qty * items.at(l.item_id).weight;
        return w;
    }

    double order_price(const Order& o) const {
        double p = 0.0;
        for (auto& l : o.lines) p += l.qty * items.at(l.item_id).price;
        return p;
    }

    double order_handle_time(const Order& o) const {
        double t = 0.0;
        for (auto& l : o.lines) {
            const Item& it = items.at(l.item_id);
            t += l.qty * (1.0 + it.handle_time);
        }
        return t;
    }

    vector<string> order_shelves(const Order& o) const {
        set<string> shelves;
        for (auto& l : o.lines) shelves.insert(items.at(l.item_id).shelf);
        return vector<string>(shelves.begin(), shelves.end());
    }

    static double normalize(double v, const FeatureRange& r) {
        double denom = max(1.0, r.max_v - r.min_v);
        return (v - r.min_v) / denom;
    }

    Plan solve_greedy(bool use_enhanced) {
        Plan plan;
        graph.floyd_warshall();

        map<string,int> total_demand;
        for (auto& o : orders)
            for (auto& l : o.lines)
                total_demand[l.item_id] += l.qty;

        map<string,int> shelf_stock;
        for (auto& [id, it] : items) shelf_stock[id] = it.stock;

        map<string,int> reserve_left;
        for (auto& [item_id, ri] : restock_info) reserve_left[item_id] = ri.reserve;

        map<string,double> last_restock_arrival;
        for (auto& [id, it] : items) last_restock_arrival[id] = 0.0;

        int rr_id = 0;
        vector<bool> restock_applied;

        auto apply_restocks_up_to = [&](double t) {
            for (size_t i = 0; i < plan.restock_requests.size(); ++i) {
                if (i < restock_applied.size() && restock_applied[i]) continue;
                if (plan.restock_requests[i].arrival_time <= t) {
                    const auto& rr = plan.restock_requests[i];
                    shelf_stock[rr.item_id] += rr.qty;
                    last_restock_arrival[rr.item_id] = max(last_restock_arrival[rr.item_id], rr.arrival_time);
                    if (i >= restock_applied.size()) restock_applied.resize(i + 1, false);
                    restock_applied[i] = true;
                }
            }
        };

        auto schedule_restock = [&](const string& item_id, int needed_qty, double creation_time, double arrival_time, double buffer) -> bool {
            if (!restock_info.count(item_id)) return false;
            auto& ri = restock_info[item_id];
            double unit_w = items[item_id].weight;
            int units_per_delivery = max(1, (int)floor(ri.max_weight / unit_w));
            int remaining = needed_qty;
            while (remaining > 0) {
                if (reserve_left[item_id] <= 0) return false;
                int deliver_qty = min(remaining, min(units_per_delivery, reserve_left[item_id]));
                RestockRequest rr;
                rr.id = rr_id++;
                rr.item_id = item_id;
                rr.qty = deliver_qty;
                rr.shelf = items[item_id].shelf;
                rr.creation_time = creation_time;
                rr.arrival_time = arrival_time;
                rr.delay_buffer_used = buffer;
                rr.latest_needed = INF;
                plan.restock_requests.push_back(rr);
                reserve_left[item_id] -= deliver_qty;
                remaining -= deliver_qty;
            }
            return true;
        };

        if (use_enhanced) {
            for (auto& [item_id, ri] : restock_info) {
                int demand = total_demand.count(item_id) ? total_demand[item_id] : 0;
                int shortfall = demand - shelf_stock[item_id];
                if (shortfall <= 0) continue;

                if (reserve_left[item_id] < shortfall) {
                    plan.feasible = false;
                    plan.infeasible_reason = "Insufficient reserve stock for item " + item_id;
                    return plan;
                }

                double unit_w = items[item_id].weight;
                int units_per_delivery = max(1, (int)floor(ri.max_weight / unit_w));
                int remaining = shortfall;
                double creation_t = 0.0;
                double buf = (ri.delay_risk > 0.3) ? ri.delay_buffer : 0.0;
                while (remaining > 0) {
                    int deliver_qty = min(remaining, units_per_delivery);
                    double arrival = creation_t + ri.prep_time + ri.delivery_time + buf;
                    RestockRequest rr;
                    rr.id = rr_id++;
                    rr.item_id = item_id;
                    rr.qty = deliver_qty;
                    rr.shelf = items[item_id].shelf;
                    rr.creation_time = creation_t;
                    rr.arrival_time = arrival;
                    rr.delay_buffer_used = buf;
                    rr.latest_needed = INF;
                    plan.restock_requests.push_back(rr);
                    reserve_left[item_id] -= deliver_qty;
                    remaining -= deliver_qty;
                    creation_t += 1.0;
                }
            }
        }

        struct ScoredOrder { Order* order; double score; };
        vector<ScoredOrder> sorted;
        map<string, double> order_score;
        FeatureRange r_priority, r_slack, r_stock, r_handle, r_price;

        for (auto& o : orders) {
            double priorityWeight = (o.priority == Priority::URGENT) ? 3.0 : (o.priority == Priority::VIP) ? 2.0 : 1.0;
            double slack = o.deadline - o.release;
            double stockUrgency = 1.0;
            for (auto& l : o.lines) {
                int demand = max(1, total_demand[l.item_id]);
                double ratio = (double)shelf_stock[l.item_id] / demand;
                stockUrgency = min(stockUrgency, 1.0 - ratio);
            }
            double expectedHandle = order_handle_time(o);
            double price = order_price(o);

            r_priority.min_v = min(r_priority.min_v, priorityWeight);
            r_priority.max_v = max(r_priority.max_v, priorityWeight);
            r_slack.min_v = min(r_slack.min_v, slack);
            r_slack.max_v = max(r_slack.max_v, slack);
            r_stock.min_v = min(r_stock.min_v, stockUrgency);
            r_stock.max_v = max(r_stock.max_v, stockUrgency);
            r_handle.min_v = min(r_handle.min_v, expectedHandle);
            r_handle.max_v = max(r_handle.max_v, expectedHandle);
            r_price.min_v = min(r_price.min_v, price);
            r_price.max_v = max(r_price.max_v, price);
        }

        for (auto& o : orders) {
            double priorityWeight = (o.priority == Priority::URGENT) ? 3.0 : (o.priority == Priority::VIP) ? 2.0 : 1.0;
            double slack = o.deadline - o.release;
            double stockUrgency = 1.0;
            for (auto& l : o.lines) {
                int demand = max(1, total_demand[l.item_id]);
                double ratio = (double)shelf_stock[l.item_id] / demand;
                stockUrgency = min(stockUrgency, 1.0 - ratio);
            }
            double expectedHandle = order_handle_time(o);
            double price = order_price(o);

            if (!use_enhanced) {
                sorted.push_back({&o, 0.0});
                order_score[o.id] = 0.0;
                continue;
            }

            double p_n = normalize(priorityWeight, r_priority);
            double s_n = normalize(slack, r_slack);
            double u_n = normalize(stockUrgency, r_stock);
            double h_n = normalize(expectedHandle, r_handle);
            double pr_n = normalize(price, r_price);

            double score = 0.35 * p_n + 0.20 * pr_n + 0.25 * u_n - 0.10 * h_n - 0.10 * s_n;
            sorted.push_back({&o, score});
            order_score[o.id] = score;
        }

        if (use_enhanced) {
            sort(sorted.begin(), sorted.end(),
                [&](const ScoredOrder& a, const ScoredOrder& b){
                    if (a.score != b.score) return a.score > b.score;
                    return a.order->deadline < b.order->deadline;
                });
        } else {
            sort(sorted.begin(), sorted.end(),
                [&](const ScoredOrder& a, const ScoredOrder& b){
                    return a.order->deadline < b.order->deadline;
                });
        }

        map<string, double> picker_free;
        for (auto& p : pickers) picker_free[p.id] = p.shift_start;

        int batch_id = 0;
        vector<bool> assigned(sorted.size(), false);

        auto has_pending_restock = [&](const string& item_id, double t) -> bool {
            for (auto& rr : plan.restock_requests)
                if (rr.item_id == item_id && rr.arrival_time > t) return true;
            return false;
        };

        auto build_batch = [&](Picker& picker, double batch_start) -> PickBatch {
            PickBatch batch;
            batch.id = batch_id++;
            batch.picker_id = picker.id;
            batch.start_time = batch_start;

            double capacity_left = picker.capacity;
            string cur_loc = "D";
            vector<int> batch_indices;

            while (true) {
                int best_idx = -1;
                double best_value = -INF;
                double max_dist = 0.0;
                struct Candidate { int idx; double dist; double score; };
                vector<Candidate> candidates;

                for (int i = 0; i < (int)sorted.size(); ++i) {
                    if (assigned[i]) continue;
                    Order& o = *sorted[i].order;
                    if (o.release > batch_start) continue;

                    double w = order_weight(o);
                    if (w > capacity_left) continue;

                    bool stock_ok = true;
                    for (auto& l : o.lines) {
                        if (shelf_stock[l.item_id] < l.qty) {
                            stock_ok = false;
                            if (!has_pending_restock(l.item_id, batch_start) && restock_info.count(l.item_id)) {
                                auto& ri = restock_info[l.item_id];
                                double buf = (ri.delay_risk > 0.3) ? ri.delay_buffer : 0.0;
                                double lead = ri.prep_time + ri.delivery_time + buf;
                                double creation_t = use_enhanced ? max(0.0, batch_start - lead) : batch_start;
                                double arrival_t = creation_t + lead;
                                schedule_restock(l.item_id, l.qty - shelf_stock[l.item_id], creation_t, arrival_t, buf);
                            }
                            break;
                        }
                    }
                    if (!stock_ok) continue;

                    bool perishable_ok = true;
                    for (auto& l : o.lines) {
                        const Item& it = items[l.item_id];
                        if (it.itype == ItemType::PERISHABLE &&
                            (batch_start - last_restock_arrival[l.item_id] > it.max_wait)) {
                            perishable_ok = false;
                            break;
                        }
                    }
                    if (!perishable_ok) continue;

                    double dist = INF;
                    for (auto& shelf : order_shelves(o))
                        dist = min(dist, graph.travel(cur_loc, shelf));
                    max_dist = max(max_dist, dist);
                    candidates.push_back({i, dist, order_score[o.id]});
                }

                if (candidates.empty()) break;

                double denom = max(1.0, max_dist);
                for (auto& c : candidates) {
                    double dist_norm = c.dist / denom;
                    double value = c.score - 0.2 * dist_norm;
                    if (value > best_value) {
                        best_value = value;
                        best_idx = c.idx;
                    }
                }

                if (best_idx < 0) break;

                Order& chosen = *sorted[best_idx].order;
                batch.order_ids.push_back(chosen.id);
                batch_indices.push_back(best_idx);
                assigned[best_idx] = true;
                capacity_left -= order_weight(chosen);

                double nearest = INF;
                string nearest_shelf = cur_loc;
                for (auto& shelf : order_shelves(chosen)) {
                    double d = graph.travel(cur_loc, shelf);
                    if (d < nearest) {
                        nearest = d;
                        nearest_shelf = shelf;
                    }
                }
                cur_loc = nearest_shelf;
            }

            if (batch.order_ids.empty()) {
                batch.id = -1;
                return batch;
            }

            auto compute_batch_timing = [&](PickBatch& b) -> bool {
                set<string> shelves_needed;
                map<string, map<string,int>> shelf_picks;
                double total_weight = 0.0;
                double max_pack = 0.0;

                for (auto& oid : b.order_ids) {
                    Order* o = nullptr;
                    for (auto& ord : orders) if (ord.id == oid) { o = &ord; break; }
                    max_pack = max(max_pack, o->pack_time);
                    total_weight += order_weight(*o);
                    for (auto& l : o->lines) {
                        shelves_needed.insert(items[l.item_id].shelf);
                        shelf_picks[items[l.item_id].shelf][l.item_id] += l.qty;
                    }
                }

                vector<string> route = graph.tsp_greedy("D", vector<string>(shelves_needed.begin(), shelves_needed.end()));
                b.shelf_sequence = route;
                b.total_weight = total_weight;

                double cur_time = b.start_time;
                string loc = "D";
                for (auto& shelf : route) {
                    cur_time += graph.travel(loc, shelf);
                    loc = shelf;
                    if (shelf_picks.count(shelf)) {
                        for (auto& [item_id, qty] : shelf_picks[shelf]) {
                            const Item& it = items[item_id];
                            if (it.itype == ItemType::PERISHABLE &&
                                (cur_time - last_restock_arrival[item_id] > it.max_wait)) {
                                return false;
                            }
                            cur_time += qty * (1.0 + it.handle_time);
                        }
                    }
                }
                cur_time += graph.travel(loc, "D");
                cur_time += max_pack;
                b.finish_time = cur_time;
                b.dispatch_time = cur_time;
                return true;
            };

            while (true) {
                if (!compute_batch_timing(batch)) {
                    int last = batch_indices.back();
                    assigned[last] = false;
                    batch.order_ids.pop_back();
                    batch_indices.pop_back();
                    if (batch.order_ids.empty()) {
                        batch.id = -1;
                        return batch;
                    }
                    continue;
                }

                double max_deadline = INF;
                for (auto& oid : batch.order_ids) {
                    for (auto& ord : orders)
                        if (ord.id == oid) max_deadline = min(max_deadline, ord.deadline);
                }
                if (batch.dispatch_time > max_deadline) {
                    int last = batch_indices.back();
                    assigned[last] = false;
                    batch.order_ids.pop_back();
                    batch_indices.pop_back();
                    if (batch.order_ids.empty()) {
                        batch.id = -1;
                        return batch;
                    }
                    continue;
                }
                break;
            }

            return batch;
        };

        bool progress = true;
        while (progress) {
            progress = false;
            for (auto& picker : pickers) {
                double batch_start = picker_free[picker.id];
                double max_end = picker.shift_end + picker.overtime_limit;
                if (batch_start > max_end) continue;

                apply_restocks_up_to(batch_start);
                PickBatch batch = build_batch(picker, batch_start);
                if (batch.id < 0) continue;

                if (batch.finish_time > max_end) {
                    plan.feasible = false;
                    plan.infeasible_reason = "Picker " + picker.id + " exceeds overtime.";
                    return plan;
                }

                for (auto& oid : batch.order_ids) {
                    for (auto& ord : orders) if (ord.id == oid) {
                        if (batch.dispatch_time > ord.deadline) {
                            plan.feasible = false;
                            plan.infeasible_reason = "Order " + ord.id + " misses deadline.";
                            return plan;
                        }
                    }
                }

                for (auto& oid : batch.order_ids) {
                    for (auto& ord : orders) if (ord.id == oid) {
                        for (auto& l : ord.lines) shelf_stock[l.item_id] -= l.qty;
                    }
                }

                plan.pick_batches.push_back(batch);
                picker_free[picker.id] = batch.finish_time;
                progress = true;
            }
        }

        for (int i = 0; i < (int)sorted.size(); ++i) {
            if (!assigned[i]) {
                plan.feasible = false;
                plan.infeasible_reason = "Order " + sorted[i].order->id + " could not be assigned.";
                return plan;
            }
        }

        return plan;
    }
};

// Output Helper

void print_plan(const Plan& plan, const WarehousePlanner& planner, const string& title) {
    if (!plan.feasible) {
        cout << "\n=== " << title << " ===\n";
        cout << "=== INFEASIBLE ===\n";
        cout << "Reason: " << plan.infeasible_reason << "\n";
        return;
    }

    cout << "\n=== " << title << " ===\n";
    cout << "=== FEASIBLE PLAN ===\n\n";

    cout << "-- Restock Requests -----------------------\n";
    if (plan.restock_requests.empty()) {
        cout << "  (none required - shelf stock sufficient)\n";
    }
    for (auto& rr : plan.restock_requests) {
        cout << "  RR-" << rr.id
             << "  item=" << rr.item_id
             << "  qty=" << rr.qty
             << "  shelf=" << rr.shelf
             << "  created at" << fixed << setprecision(1) << rr.creation_time
             << "  arrives at" << rr.arrival_time;
        if (rr.delay_buffer_used > 0)
            cout << "  [buffer=" << rr.delay_buffer_used << "min]";
        cout << "\n";
    }

    cout << "\n-- Pick Batches -----------------------\n";
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
        for (auto& s : b.shelf_sequence) cout << " -> " << s;
        cout << " -> D\n";
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
            it.price  = 10 + rnd(0, 40);
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
        it.price = 8 + rnd(0, 30);
        wp.items[iid] = it;
    }
    // Item X — heavily understocked
    wp.items["IX"] = {"IX","S10", 5, 1.5, ItemType::NORMAL, 1e18, 0.0, 20.0};

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
        it.price = 6 + rnd(0, 25);
        wp.items[iid] = it;
    }
    // Three understocked items
    for (auto& [nm, sh, stk, w] : vector<tuple<string,string,int,double>>{
            {"IA","S5",3,2.0},{"IB","S20",2,1.5},{"IC","S55",4,1.0}}) {
        wp.items[nm] = {nm, sh, stk, w, ItemType::NORMAL, 1e18, 0.0, 18.0};
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
        it.price = 9 + rnd(0, 30);
        wp.items[iid] = it;
    }
    // VIP item — understocked; restock arrives at 80 min but VIP deadline is 60 min
    wp.items["IVIP"] = {"IVIP","S25", 0, 1.0, ItemType::NORMAL, 1e18, 0.0, 25.0};

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

WarehousePlanner build_case5() {
    // Case 5: all remaining orders need IX, shelf stock is zero,
    // and the restock arrives later. This exposes the stall where
    // no batch can be built before the restock arrives.
    mt19937 rng(SEED);
    WarehousePlanner wp;

    vector<string> nodes = {"D", "V", "S1"};
    wp.graph.add_nodes(nodes);

    wp.graph.add_edge("D", "V", 10);
    wp.graph.add_edge("D", "S1", 5);
    wp.graph.add_edge("V", "S1", 5);
    wp.graph.floyd_warshall();

    Item ix;
    ix.id = "IX";
    ix.shelf = "S1";
    ix.stock = 0;
    ix.weight = 1.0;
    ix.itype = ItemType::NORMAL;
    ix.price = 20.0;
    wp.items["IX"] = ix;

    wp.pickers = {
        {"P1", 0, 480, 50},
    };

    for (int o = 1; o <= 3; ++o) {
        Order ord;
        ord.id = "O" + to_string(o);
        ord.release = 0;
        ord.deadline = 200;
        ord.priority = Priority::NORMAL;
        ord.pack_time = 2.0;
        ord.lines.push_back({"IX", 5});
        wp.orders.push_back(ord);
    }

    RestockInfo ri;
    ri.item_id = "IX";
    ri.reserve = 100;
    ri.prep_time = 20;
    ri.delivery_time = 60;   // arrives at 80
    ri.max_weight = 50;
    ri.delay_risk = 0.0;
    ri.delay_buffer = 0.0;
    wp.restock_info["IX"] = ri;

    return wp;
}

// Main

int main(int argc, char* argv[]) {
    int case_num = 1;
    if (argc > 1) case_num = atoi(argv[1]);

    cout << "=== Warehouse Operations Planner - Thread A (Human) ===\n";
    cout << "    Running Case " << case_num << "\n";

    WarehousePlanner wp;
    switch (case_num) {
        case 1: wp = build_case1(); break;
        case 2: wp = build_case2(); break;
        case 3: wp = build_case3(); break;
        case 4: wp = build_case4(); break;
        case 5: wp = build_case5(); break;
        default:
            cerr << "Unknown case. Use 1-5.\n";
            return 1;
    }

    auto start = chrono::high_resolution_clock::now();
    Plan baseline = wp.solve_baseline();
    Plan enhanced = wp.solve_enhanced();
    auto end   = chrono::high_resolution_clock::now();
    double ms  = chrono::duration<double,milli>(end-start).count();

    print_plan(baseline, wp, "Baseline Greedy (Deadline Only, Naive Restock)");
    print_plan(enhanced, wp, "Enhanced Greedy (Score + Lead-Time Restock)");
    cout << "Runtime: " << fixed << setprecision(2) << ms << " ms\n";
    return 0;
}
