import gurobi.*;
import gurobi.GRB.IntParam;

import java.io.*;
import java.util.ArrayList;
import java.util.Arrays;

public class Gurobi {

    private static final int N_VERTICI = 43;

    public static void main(String[] args) throws IOException {
        //lettura file txt
         /** //gruppo68
         int v = 13;
         int a = 5;
         int b1 = 15;
         int b2 = 5;
         int c = 134;
         int d1 = 27;
         int d2 = 36;
         int e1 = 11;
         int e2 = 5;
         int f1 = 34;
         int f2 = 38;
         int g1 = 46;
         int g2 = 0;
         int h1 = 0;
         int h2 = 32;
         int i1  = 4;
         int i2 = 21;
         int l = 3;
        **/
        /** //gruppo25
        int v = 2;
        int a = 7;
        int b1 = 6;
        int b2 = 25;
        int c = 114;
        int d1 = 36;
        int d2 = 1;
        int e1 = 13;
        int e2 = 10;
        int f1 = 32;
        int f2 = 2;
        int g1 = 2;
        int g2 = 8;
        int h1 = 16;
        int h2 = 39;
        int i1  = 7;
        int i2 = 9;
        int l = 2;
        **/
        //gruppo16
        int v = 34;
        int a = 9;
        int b1 = 13;
        int b2 = 25;
        int c = 138;
        int d1 = 0;
        int d2 = 14;
        int e1 = 27;
        int e2 = 24;
        int f1 = 38;
        int f2 = 39;
        int g1 = 40;
        int g2 = 26;
        int h1 = 33;
        int h2 = 32;
        int i1  = 10;
        int i2 = 7;
        int l = 7;

        String file_name = "src/coppia16.txt";

        int[][] costi = new int[N_VERTICI][N_VERTICI];
        parsing_file(file_name, costi);

        try{

            GRBEnv env = new GRBEnv("Coppia16.log");
            env.set(IntParam.Presolve, 0);
            env.set(IntParam.Method, 0);


            //---------------------------------------QUESITO I---------------------------------------------------

            //modello 1 gurobi
            GRBModel model = new GRBModel(env);

            //creazione variabili Xij
            GRBVar[][] Xij = new GRBVar[N_VERTICI][N_VERTICI];
            for(int i=0; i< N_VERTICI; i++){
                for(int j=0; j< N_VERTICI; j++) {
                    Xij[i][j] = model.addVar(0.0, 1.0, 0.0, GRB.BINARY, "X_" + i + "_" + j);
                }
            }
            //creazione variabili u di supporto
            GRBVar[] u = new GRBVar[N_VERTICI];
            for (int m=0; m<N_VERTICI; m++){
                u[m] = model.addVar(0.0, N_VERTICI, 0.0, GRB.INTEGER, "u "+ u);
            }

            //---------------------------------------F.O.----------------------------------------

            GRBLinExpr expr = new GRBLinExpr();
            for(int i=0; i< N_VERTICI; i++) {
                for (int j = 0; j < N_VERTICI; j++) {
                    expr.addTerm(costi[i][j], Xij[i][j]);
                }
            }
            model.setObjective(expr);
            model.set(GRB.IntAttr.ModelSense, GRB.MINIMIZE);

            //-------------------------------------VINCOLI---------------------------------------

            //vincoli di assegnamento:
            //una sola uscita attiva per ogni vertice
            for(int i= 0; i< N_VERTICI; i++){
                expr = new GRBLinExpr();
                for(int j=0; j<N_VERTICI; j++){
                    expr.addTerm(1, Xij[i][j]);
                }
                model.addConstr(expr, GRB.EQUAL, 1, "una sola uscita per ogni vertice");
            }
            //una sola entrata attiva per ogni vertice
            for(int i= 0; i< N_VERTICI; i++){
                expr = new GRBLinExpr();
                for(int j=0; j<N_VERTICI; j++){
                    expr.addTerm(1, Xij[j][i]);
                }
                model.addConstr(expr, GRB.EQUAL, 1, "una sola entrata per ogni vertice");
            }

            //aggiungo N vincoli che impongano che il ciclo non si fermi in un nodo
            for(int i= 0; i< N_VERTICI; i++){
                expr = new GRBLinExpr();
                expr.addTerm(1, Xij[i][i]);
                model.addConstr(expr, GRB.EQUAL, 0, "diagonale nulla");
            }

            //aggiungo N*(N-1)/2 vincoli che impongano che il verso entrante e il verso uscente per ogni Xij non sia il medesimo
            //scorro la triangolare superiore della matrice
            for(int i = 0; i< N_VERTICI-1; i++){
                for(int j = i+1; j<N_VERTICI; j++){
                    expr = new GRBLinExpr();
                    expr.addTerm(1, Xij[i][j]);
                    expr.addTerm(1, Xij[j][i]);
                    model.addConstr(expr, GRB.LESS_EQUAL, 1, "al più un collegamento attivo tra due nodi(non entrambi andata e ritorno)");
                }
            }

            //vincoli di Miller-Tucker-Zemil utilizzando le variabili u di supporto
            //u[0] = 1
            expr = new GRBLinExpr();
            expr.addTerm(1,u[0]);
            model.addConstr(expr, GRB.EQUAL, 1, "assegnazione u[1]");
            //2 <= u[i] <= N con i= 1...N-1
            for (int i=1; i<N_VERTICI; i++){
                expr = new GRBLinExpr();
                expr.addTerm(1,u[i]);
                model.addConstr(expr, GRB.GREATER_EQUAL, 2, "limite inferiore di 2");
                expr = new GRBLinExpr();
                expr.addTerm(1,u[i]);
                model.addConstr(expr, GRB.LESS_EQUAL, N_VERTICI, "limite superiore di " + (N_VERTICI-1));
            }
            //u[j] >= u[i] + 1 - ((N-1)-1)(1-xij) con  i != j  e  i,j = 1...N-1
            //svolgendo il calcolo: u[j] >= u[i] + 2 - (N-1) + xij((N-1)-1)
            for(int i=1; i<N_VERTICI-1; i++){
                for(int j=1; j<N_VERTICI; j++){
                    expr = new GRBLinExpr();
                    if(i!=j) {
                        expr.addTerm((N_VERTICI - 1) - 1 , Xij[i][j]);
                        expr.addTerm(-1, u[j]);
                        expr.addTerm(1, u[i]);
                        model.addConstr(expr, GRB.LESS_EQUAL, (N_VERTICI- 1) - 2 , "vincolo di sequenzialità delle variabili u");
                    }
                }
            }

            //ottimizzazione modello gurobi
            model.optimize();

            //salvataggio e display del valore della funzione obiettivo
            double funzione_obiettivo_1 = model.get(GRB.DoubleAttr.ObjVal);

            //calcolo ciclo modello 1
            ArrayList<Integer> ciclo1 = calcola_ciclo(Xij);

            //-------------------------------------QUESITO II---------------------------------------

            //creo un nuovo modello identico al primo e aggiungo il vincolo che impone il raggiungimento
            // dello stesso risultato della funzione obiettivo del quesito 1

            GRBModel model2 = new GRBModel(env);
            //creazione variabili
            Xij = new GRBVar[N_VERTICI][N_VERTICI];
            for(int i=0; i< N_VERTICI; i++){
                for(int j=0; j< N_VERTICI; j++) {
                    Xij[i][j] = model2.addVar(0.0, 1.0, 0.0, GRB.BINARY, "X_" + i + "_" + j);
                }
            }
            u = new GRBVar[N_VERTICI];
            for (int m=0; m<N_VERTICI; m++){
                u[m] = model2.addVar(0.0, N_VERTICI, 0.0, GRB.INTEGER, "u "+ u);
            }

            //---------------------------------------F.O.----------------------------------------

            expr = new GRBLinExpr();
            for(int i=0; i< N_VERTICI; i++) {
                for (int j = 0; j < N_VERTICI; j++) {
                    expr.addTerm(costi[i][j], Xij[i][j]);
                }
            }
            model2.setObjective(expr);
            model2.set(GRB.IntAttr.ModelSense, GRB.MINIMIZE);

            //----------------------------------------VINCOLI--------------------------------------

            //stessi vincoli del modello 1
            for(int i= 0; i< N_VERTICI; i++){
                expr = new GRBLinExpr();
                for(int j=0; j<N_VERTICI; j++){
                    expr.addTerm(1, Xij[i][j]);
                }
                model2.addConstr(expr, GRB.EQUAL, 1, "una sola uscita per ogni vertice");
            }
            for(int i= 0; i< N_VERTICI; i++){
                expr = new GRBLinExpr();
                for(int j=0; j<N_VERTICI; j++){
                    expr.addTerm(1, Xij[j][i]);
                }
                model2.addConstr(expr, GRB.EQUAL, 1, "una sola entrata per ogni vertice");
            }
            for(int i= 0; i< N_VERTICI; i++){
                expr = new GRBLinExpr();
                expr.addTerm(1, Xij[i][i]);
                model2.addConstr(expr, GRB.EQUAL, 0, "diagonale nulla");
            }
            for(int i = 0; i< N_VERTICI-1; i++){
                for(int j = i+1; j<N_VERTICI; j++){
                    expr = new GRBLinExpr();
                    expr.addTerm(1, Xij[i][j]);
                    expr.addTerm(1, Xij[j][i]);
                    model2.addConstr(expr, GRB.LESS_EQUAL, 1, "al più un collegamento attivo tra due nodi(non entrambi andata e ritorno)");
                }
            }
            expr = new GRBLinExpr();
            expr.addTerm(1,u[0]);
            model2.addConstr(expr, GRB.EQUAL, 1, "assegnazione u[1]");
            for (int i=1; i<N_VERTICI; i++){
                expr = new GRBLinExpr();
                expr.addTerm(1,u[i]);
                model2.addConstr(expr, GRB.GREATER_EQUAL, 2, "limite inferiore di 2");
                expr = new GRBLinExpr();
                expr.addTerm(1,u[i]);
                model2.addConstr(expr, GRB.LESS_EQUAL, N_VERTICI, "limite superiore di " + N_VERTICI);
            }
            for(int i=1; i<N_VERTICI-1; i++){
                for(int j=1; j<N_VERTICI; j++){
                    expr = new GRBLinExpr();
                    if(i!=j) {
                        expr.addTerm((N_VERTICI - 1) - 1 , Xij[i][j]);
                        expr.addTerm(-1, u[j]);
                        expr.addTerm(1, u[i]);
                        model2.addConstr(expr, GRB.LESS_EQUAL, (N_VERTICI - 1) - 2 , "vincolo di sequenzialità delle variabili u");
                    }
                }
            }

            //Vincolo aggiunto rispetto al quesito I.
            //Impone che la funzione obiettivo sia pari a quella del primo quesito
            expr = new GRBLinExpr();
            for(int i=0; i< N_VERTICI; i++) {
                for (int j = 0; j < N_VERTICI; j++) {
                    expr.addTerm(costi[i][j], Xij[i][j]);
                }
            }
            model2.addConstr(expr, GRB.EQUAL, funzione_obiettivo_1, "valori funzioni obiettivo uguali");

            //ottimizzazione
            model2.optimize();

            //calcolo ciclo 2
            ArrayList<Integer> ciclo2 = calcola_ciclo(Xij);

            //-----------------------------------QUESITO III---------------------------------

            //creo un nuovo modello identico al modello 1 e scrivo i vincoli aggiuntivi
            GRBModel model3 = new GRBModel(env);
            //creazione variabili
            Xij = new GRBVar[N_VERTICI][N_VERTICI];
            for(int i=0; i< N_VERTICI; i++){
                for(int j=0; j< N_VERTICI; j++) {
                    Xij[i][j] = model3.addVar(0.0, 1.0, 0.0, GRB.BINARY, "X_" + i + "_" + j);
                }
            }
            u = new GRBVar[N_VERTICI];
            for (int m=0; m<N_VERTICI; m++){
                u[m] = model3.addVar(0.0, N_VERTICI, 0.0, GRB.INTEGER, "u "+ u);
            }

            //---------------------------------------F.O.----------------------------------------

            expr = new GRBLinExpr();
            for(int i=0; i< N_VERTICI; i++) {
                for (int j = 0; j < N_VERTICI; j++) {
                    expr.addTerm(costi[i][j], Xij[i][j]);
                }
            }
            model3.setObjective(expr);
            model3.set(GRB.IntAttr.ModelSense, GRB.MINIMIZE);

            //----------------------------------------VINCOLI--------------------------------------

            //stessi vincoli del modello 1
            for(int i= 0; i< N_VERTICI; i++){
                expr = new GRBLinExpr();
                for(int j=0; j<N_VERTICI; j++){
                    expr.addTerm(1, Xij[i][j]);
                }
                model3.addConstr(expr, GRB.EQUAL, 1, "una sola uscita per ogni vertice");
            }
            for(int i= 0; i< N_VERTICI; i++){
                expr = new GRBLinExpr();
                for(int j=0; j<N_VERTICI; j++){
                    expr.addTerm(1, Xij[j][i]);
                }
                model3.addConstr(expr, GRB.EQUAL, 1, "una sola entrata per ogni vertice");
            }
            for(int i= 0; i< N_VERTICI; i++){
                expr = new GRBLinExpr();
                expr.addTerm(1, Xij[i][i]);
                model3.addConstr(expr, GRB.EQUAL, 0, "diagonale nulla");
            }
            for(int i = 0; i< N_VERTICI-1; i++){
                for(int j = i+1; j<N_VERTICI; j++){
                    expr = new GRBLinExpr();
                    expr.addTerm(1, Xij[i][j]);
                    expr.addTerm(1, Xij[j][i]);
                    model3.addConstr(expr, GRB.LESS_EQUAL, 1, "al più un collegamento attivo tra due nodi(non entrambi andata e ritorno)");
                }
            }
            expr = new GRBLinExpr();
            expr.addTerm(1,u[0]);
            model3.addConstr(expr, GRB.EQUAL, 1, "assegnazione u[1]");
            for (int i=1; i<N_VERTICI; i++){
                expr = new GRBLinExpr();
                expr.addTerm(1,u[i]);
                model3.addConstr(expr, GRB.GREATER_EQUAL, 2, "limite inferiore di 2");
                expr = new GRBLinExpr();
                expr.addTerm(1,u[i]);
                model3.addConstr(expr, GRB.LESS_EQUAL, N_VERTICI, "limite superiore di " + N_VERTICI);
            }
            for(int i=1; i<N_VERTICI-1; i++){
                for(int j=1; j<N_VERTICI; j++){
                    expr = new GRBLinExpr();
                    if(i!=j) {
                        expr.addTerm((N_VERTICI - 1) - 1 , Xij[i][j]);
                        expr.addTerm(-1, u[j]);
                        expr.addTerm(1, u[i]);
                        model3.addConstr(expr, GRB.LESS_EQUAL, (N_VERTICI - 1) - 2 , "vincolo di sequenzialità delle variabili u");
                    }
                }
            }
            /**
            //VINCOLI AGGIUNTIVI
            //il costo dei lati incidenti a v sia al massimo il a% del costo totale del ciclo
            int costo_lati_incidenti = 0;
            expr = new GRBLinExpr();
            for(int i=0; i< N_VERTICI; i++) {
                for (int j = 0; j < N_VERTICI; j++) {
                    expr.addTerm((a/100)*costi[i][j], Xij[i][j]);
                    if(Xij[v][j].get(GRB.DoubleAttr.X)==1)
                        costo_lati_incidenti+=costi[v][j];
                    if(Xij[i][v].get(GRB.DoubleAttr.X)==1)
                        costo_lati_incidenti+=costi[i][v];
                }
            }
            model3.addConstr(expr, GRB.GREATER_EQUAL, costo_lati_incidenti, "il costo dei lati incidenti a v sia al massimo il a% del costo");

            //se il lato (b1,b2) viene percorso, il costo del ciclo ottimo sia inferiore a c
            if(Xij[b1][b2].get(GRB.DoubleAttr.X)==1 || Xij[b2][b1].get(GRB.DoubleAttr.X)==1){
                expr = new GRBLinExpr();
                for(int i=0; i< N_VERTICI; i++) {
                    for (int j = 0; j < N_VERTICI; j++) {
                        expr.addTerm(costi[i][j], Xij[i][j]);
                    }
                }
                model3.addConstr(expr, GRB.LESS_EQUAL, c,"se il lato b1 b2 viene percorso, il costo del ciclo sia inferiore a c");
            }
            **/
            //il lato (d1, d2) sia percorribile se e solo se sono percorsi anche i lati (e1, e2) e (f1, f2)
            //2*Xij[d1][d2] - Xij[e1][e2] - Xij[f1][f2] <= 0
            expr = new GRBLinExpr();
            expr.addTerm(2,Xij[d1][d2]);
            expr.addTerm(-1,Xij[f1][f2]);
            expr.addTerm(-1,Xij[e1][e2]);
            model3.addConstr(expr, GRB.LESS_EQUAL, 0, "il lato (d1, d2) sia percorribile se e solo se sono percorsi anche i lati (e1, e2) e (f1, f2)");
            /**
            //nel caso in cui i lati (g1, g2), (h1, h2) e (i1, i2) vengano tutti percorsi, si debba pagare un costo aggiuntivo pari a l.
            //if(Xij[e1][e2].get(GRB.DoubleAttr.X)==0)
            **/
            //ottimizzazione
            model3.optimize();

            //salvataggio e display del valore della funzione obiettivo
            double funzione_obiettivo_3 = model3.get(GRB.DoubleAttr.ObjVal);

            //calcolo ciclo 3
            ArrayList<Integer> ciclo3 = calcola_ciclo(Xij);

            stampa_finale((int)funzione_obiettivo_1, ciclo1, ciclo2, (int)funzione_obiettivo_3, ciclo3);

        }catch(GRBException e){
            System.out.println("Error code: " + e.getErrorCode() + ". " + e.getMessage());
        }
    }

    public static void parsing_file(String file_name, int[][] costi) throws IOException {

        File file = new File(file_name);
        BufferedReader br = new BufferedReader(new FileReader(file));
        String lettura;
        boolean start_parse = false;

        String parola_start_read = "Vertici " + N_VERTICI;

        while((lettura = br.readLine()) != null){
            if(start_parse){
                int[] numeri_letti = Arrays.stream(lettura.split(" ")).mapToInt(Integer::parseInt).toArray();
                costi[numeri_letti[0]][numeri_letti[1]] = costi[numeri_letti[1]][numeri_letti[0]]= numeri_letti[2];
            }
            if(lettura.equals(parola_start_read)){
                start_parse = true;
            }
        }
        //imposto la diagonale della matrice pari a 0; il costo da un nodo a sè stesso è pari a 0.
        for(int i= 0; i < N_VERTICI; i++){
            costi[i][i] = 0;
        }
    }

    public static void stampa_finale(int funzione_obiettivo_1, ArrayList<Integer> ciclo1, ArrayList<Integer> ciclo2, int funzione_obiettivo_3, ArrayList<Integer> ciclo3){
        //---------------------STAMPA A VIDEO-------------------------
        System.out.printf("\n\n\n");
        System.out.println("GRUPPO <coppia 16>");
        System.out.println("Componenti: <Bresciani Simone> <Dagani Federico>\n");
        System.out.println("QUESITO I:");
        System.out.printf("funzione obiettivo = %d\n", (int)funzione_obiettivo_1);
        System.out.print("ciclo ottimo 1: ");
        System.out.println(ciclo1);
        System.out.println("\nQUESITO II:");
        System.out.print("ciclo ottimo 2: ");
        System.out.println(ciclo2);
        System.out.println("\nQUESITO III:");
        System.out.printf("funzione obiettivo = %d\n", (int)funzione_obiettivo_3);
        System.out.print("ciclo ottimo 3: ");
        System.out.println(ciclo3);
    }

    public static ArrayList<Integer> calcola_ciclo(GRBVar[][] Xij) throws GRBException {

        ArrayList<Integer> ciclo = new ArrayList<>();
        int precedente=0;
        ciclo.add(precedente);
        for(int i=0; i<N_VERTICI; i++){
            for(int k= 0; k< N_VERTICI; k++) {
                if (Xij[precedente][k].get(GRB.DoubleAttr.X) == 1) {
                    ciclo.add(k);
                    precedente=k;
                    break;
                }
            }
        }
        return ciclo;
    }

    //potrei fare una funzione che restituisce il "modello base", ovvero quello utilizzato da tutti e tre i quesiti
}
